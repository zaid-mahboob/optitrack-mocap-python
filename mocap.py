"""
OptiTrack NatNet Python client.

All tunable parameters (IPs, capture rate, body IDs) live in config.py.
Edit that file — nothing here needs to change for normal use.

Quickstart
----------
    from mocap import MocapClient
    from config import CLIENT_ADDRESS, SERVER_ADDRESS, BODY_IDS

    client = MocapClient(client_address=CLIENT_ADDRESS,
                         server_address=SERVER_ADDRESS)
    client.start()

    pos, quat, linvel, angvel = client.get_body_state(BODY_IDS["left_foot"])
    client.shutdown()

Return values
-------------
    pos          (x, y, z)          metres, MoCap world frame
    quat         (qw, qx, qy, qz)   unit quaternion
    linvel        (vx, vy, vz)       m/s,   averaged over VELOCITY_WINDOW_S
    angvel        (wx, wy, wz)       rad/s, averaged over VELOCITY_WINDOW_S
"""

import math
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Load configuration — all tunable values come from config.py
# ---------------------------------------------------------------------------
_here = Path(__file__).resolve().parent
sys.path.insert(0, str(_here))

from config import (  # noqa: E402
    CLIENT_ADDRESS,
    SERVER_ADDRESS,
    USE_MULTICAST,
    VELOCITY_RATE_HZ,
    VELOCITY_WINDOW_S,
    VELOCITY_HALF_S,
    MAX_SAMPLES_PER_BODY,
)

# ---------------------------------------------------------------------------
# NatNet SDK — bundled in natnet_sdk/, override with NATNET_SDK_PATH env var
# ---------------------------------------------------------------------------
NATNET_SDK_PATH = os.environ.get(
    "NATNET_SDK_PATH",
    str(_here / "natnet_sdk"),
)
if NATNET_SDK_PATH and NATNET_SDK_PATH not in sys.path:
    sys.path.insert(0, NATNET_SDK_PATH)

try:
    from NatNetClient import NatNetClient  # type: ignore[import-untyped]
    try:
        NatNetClient.print_level = 0
    except Exception:
        pass
except ImportError as e:
    NatNetClient = None
    _natnet_import_error = str(e)


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

def _quat_inv(q: Tuple) -> Tuple:
    return (q[0], -q[1], -q[2], -q[3])


def _quat_mult(a: Tuple, b: Tuple) -> Tuple:
    w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return (w, x, y, z)


def _quat_mean(quats: list) -> Tuple:
    if not quats:
        return (1.0, 0.0, 0.0, 0.0)
    ref = quats[0]
    aligned = []
    for q in quats:
        if ref[0]*q[0] + ref[1]*q[1] + ref[2]*q[2] + ref[3]*q[3] < 0:
            q = (-q[0], -q[1], -q[2], -q[3])
        aligned.append(q)
    n = len(aligned)
    w = sum(q[0] for q in aligned) / n
    x = sum(q[1] for q in aligned) / n
    y = sum(q[2] for q in aligned) / n
    z = sum(q[3] for q in aligned) / n
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/norm, x/norm, y/norm, z/norm)


def _quat_to_angular_velocity(q_rel: Tuple, dt: float) -> Tuple:
    qw, qx, qy, qz = q_rel
    if qw < 0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz
    qw = max(-1.0, min(1.0, qw))
    angle = 2.0 * math.acos(qw)
    if angle < 1e-9 or dt < 1e-9:
        return (0.0, 0.0, 0.0)
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-9:
        return (0.0, 0.0, 0.0)
    scale = angle / (s * dt)
    return (qx * scale, qy * scale, qz * scale)


# ---------------------------------------------------------------------------
# Public conversion utilities
# ---------------------------------------------------------------------------

def quat_to_euler_xyz(q: Tuple) -> Tuple:
    """(qw,qx,qy,qz) → (roll, pitch, yaw) radians, XYZ convention."""
    w, x, y, z = q
    sinp = 2 * (w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return (roll, pitch, yaw)


def quat_to_rotmat(q: Tuple) -> np.ndarray:
    """(qw,qx,qy,qz) → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),    1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),   1-2*(x*x+y*y)],
    ], dtype=np.float64)


def euler_xyz_to_quat(roll: float, pitch: float, yaw: float) -> Tuple:
    """(roll, pitch, yaw) radians → (qw, qx, qy, qz)."""
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return (
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    )


# ---------------------------------------------------------------------------
# MocapClient
# ---------------------------------------------------------------------------

class MocapClient:
    """
    Streams rigid body state from an OptiTrack Motive server via NatNet.

    Parameters are read from config.py by default. You can override any of
    them per-instance if needed (e.g. for testing with a different server).
    """

    def __init__(
        self,
        client_address:   str  = CLIENT_ADDRESS,
        server_address:   str  = SERVER_ADDRESS,
        use_multicast:    bool = USE_MULTICAST,
        velocity_rate_hz: int  = VELOCITY_RATE_HZ,
    ) -> None:
        if NatNetClient is None:
            raise ImportError(
                f"NatNet SDK not found: {_natnet_import_error}\n"
                "Set NATNET_SDK_PATH or keep natnet_sdk/ next to mocap.py."
            )
        self._client_address   = client_address
        self._server_address   = server_address
        self._use_multicast    = use_multicast
        self._velocity_loop_dt = 1.0 / velocity_rate_hz
        self._lock             = threading.Lock()
        self._body_buffers:  dict[int, deque] = {}
        self._state_cache:   dict[int, dict]  = {}
        self._client:        Optional[NatNetClient] = None
        self._stop_event     = threading.Event()
        self._velocity_thread: Optional[threading.Thread] = None

    def _on_new_frame(self, data_dict: dict) -> None:
        mocap_data = data_dict.get("mocap_data")
        if mocap_data is None or mocap_data.rigid_body_data is None:
            return
        t = time.monotonic()
        with self._lock:
            for rb in mocap_data.rigid_body_data.rigid_body_list:
                bid  = rb.id_num
                pos  = (float(rb.pos[0]), float(rb.pos[1]), float(rb.pos[2]))
                # NatNet quaternion convention: (qx, qy, qz, qw) → convert to (qw, qx, qy, qz)
                qx, qy, qz, qw = (float(rb.rot[0]), float(rb.rot[1]),
                                   float(rb.rot[2]), float(rb.rot[3]))
                quat = (qw, qx, qy, qz)

                if bid not in self._body_buffers:
                    self._body_buffers[bid] = deque(maxlen=MAX_SAMPLES_PER_BODY)
                self._body_buffers[bid].append((t, pos, quat))

                # Write pos/quat at full NatNet frame rate — no background thread needed.
                # Velocity is filled in separately by _velocity_tick.
                if bid not in self._state_cache:
                    self._state_cache[bid] = {
                        "position":         pos,
                        "orientation":      quat,
                        "linear_velocity":  (0.0, 0.0, 0.0),
                        "angular_velocity": (0.0, 0.0, 0.0),
                    }
                else:
                    self._state_cache[bid]["position"]    = pos
                    self._state_cache[bid]["orientation"] = quat

    def _velocity_tick(self) -> None:
        with self._lock:
            for bid, buf in list(self._body_buffers.items()):
                if not buf:
                    continue
                _, pos, quat = buf[-1]
                vx = vy = vz = wx = wy = wz = 0.0
                t_now = time.monotonic()
                t_mid = t_now - VELOCITY_HALF_S
                t_old = t_now - VELOCITY_WINDOW_S
                h1 = [(t, p, q) for t, p, q in buf if t_old <= t < t_mid]
                h2 = [(t, p, q) for t, p, q in buf if t_mid <= t <= t_now]
                if h1 and h2:
                    p1 = tuple(sum(s[i] for _, s, _ in h1) / len(h1) for i in range(3))
                    p2 = tuple(sum(s[i] for _, s, _ in h2) / len(h2) for i in range(3))
                    vx = (p2[0] - p1[0]) / VELOCITY_HALF_S
                    vy = (p2[1] - p1[1]) / VELOCITY_HALF_S
                    vz = (p2[2] - p1[2]) / VELOCITY_HALF_S
                    q_rel = _quat_mult(_quat_mean([q for _, _, q in h2]),
                                       _quat_inv(_quat_mean([q for _, _, q in h1])))
                    wx, wy, wz = _quat_to_angular_velocity(q_rel, VELOCITY_HALF_S)
                self._state_cache[bid] = {
                    "position":         pos,
                    "orientation":      quat,
                    "linear_velocity":  (vx, vy, vz),
                    "angular_velocity": (wx, wy, wz),
                }

    def _velocity_loop(self) -> None:
        while not self._stop_event.wait(timeout=self._velocity_loop_dt):
            self._velocity_tick()

    def start(self) -> bool:
        """Connect to Motive and start streaming. Returns True on success."""
        for addr in [self._client_address, "0.0.0.0"]:
            client = NatNetClient()
            try:
                client.set_print_level(0)
            except Exception:
                try:
                    client.print_level = 0
                except Exception:
                    pass
            client.set_client_address(addr)
            client.set_server_address(self._server_address)
            client.set_use_multicast(self._use_multicast)
            client.new_frame_with_data_listener = self._on_new_frame
            if not client.run("d"):
                continue
            if addr != self._client_address:
                print(f"[MocapClient] Bound to {addr!r} "
                      f"(configured {self._client_address!r} unavailable).")
            self._client = client
            self._stop_event.clear()
            self._velocity_thread = threading.Thread(
                target=self._velocity_loop, daemon=True)
            self._velocity_thread.start()
            return True
        return False

    def shutdown(self, timeout: float = 2.0) -> None:
        """Stop streaming and release resources."""
        self._stop_event.set()
        if self._velocity_thread:
            self._velocity_thread.join(timeout=0.5)
            self._velocity_thread = None
        if self._client:
            def _stop():
                try:
                    self._client.shutdown()
                except Exception:
                    pass
            t = threading.Thread(target=_stop, daemon=True)
            t.start()
            t.join(timeout=timeout)
            self._client = None

    def get_body_state(
        self, body_id: int
    ) -> Optional[Tuple[Tuple, Tuple, Tuple, Tuple]]:
        """
        Return (pos, quat_wxyz, linear_vel, angular_vel) or None if not seen.
        """
        with self._lock:
            data = self._state_cache.get(body_id)
        if data is None:
            return None
        return (data["position"], data["orientation"],
                data["linear_velocity"], data["angular_velocity"])

    def get_body_pose(self, body_id: int) -> Optional[Tuple[Tuple, Tuple]]:
        """Return (pos, quat_wxyz) or None if not seen."""
        state = self.get_body_state(body_id)
        return (state[0], state[1]) if state else None
