# optitrack-python

Minimal Python client for the **OptiTrack NatNet** protocol.  
Get full 6-DOF rigid body state — position, orientation, linear velocity, angular velocity — from any Motive server with three lines of code.

No ROS. No heavy dependencies. One config file controls everything.

---

## Quick start

**1. Edit `config.py`** — set your IPs, body IDs, and capture rate (see [Configuration](#configuration) below).

**2. Run:**

```python
from mocap import MocapClient
from config import CLIENT_ADDRESS, SERVER_ADDRESS, BODY_IDS

client = MocapClient(client_address=CLIENT_ADDRESS,
                     server_address=SERVER_ADDRESS)
client.start()

pos, quat, linvel, angvel = client.get_body_state(BODY_IDS["left_foot"])
#  pos    → (x, y, z)          metres, MoCap world frame
#  quat   → (qw, qx, qy, qz)   unit quaternion
#  linvel → (vx, vy, vz)        m/s
#  angvel → (wx, wy, wz)        rad/s

client.shutdown()
```

**3. Verify your frequency:**

```bash
python examples/frequency_probe.py --name left_foot
```

---

## Repository layout

```
optitrack_mocap_python/
├── config.py             ← EDIT THIS — all settings in one place
├── mocap.py              ← client library (no edits needed for normal use)
├── natnet_sdk/           ← bundled NatNet 4.4 Python SDK (no install required)
│   ├── NatNetClient.py
│   ├── MoCapData.py
│   └── DataDescriptions.py
├── examples/
│   ├── read_body.py      ← live pose printout
│   └── frequency_probe.py← measure actual NatNet publish rate
└── README.md
```

---

## Requirements

```
numpy
```

Only one pip dependency. The NatNet SDK is pure Python and bundled in `natnet_sdk/`.

---

## Configuration

**`config.py` is the only file you need to edit.** Every tunable parameter has a comment explaining what it does and where to find the right value.

### Step 1 — Set IPs

```python
# config.py

CLIENT_ADDRESS = "169.254.160.50"   # IP of THIS machine on the MoCap network card
SERVER_ADDRESS = "169.254.160.46"   # IP of the machine running Motive
```

| Parameter | How to find it |
|---|---|
| `CLIENT_ADDRESS` | Run `ip addr` on your machine. Look for the NIC connected to the OptiTrack switch — usually a `169.254.x.x` address. |
| `SERVER_ADDRESS` | In Motive on the server PC: **Edit → Settings → Network → Local Interface** |

Both machines must be on the same subnet (same first three octets, e.g. `169.254.160.x`).

### Step 2 — Set streaming mode

```python
USE_MULTICAST = False   # False = unicast (default, recommended for one client)
                        # True  = multicast (needed for multiple simultaneous clients)
```

In Motive: **Edit → Settings → Streaming → Transmission Type** must match.

### Step 3 — Name your rigid bodies

```python
BODY_IDS = {
    "left_foot":  100,
    "right_foot": 101,
    "object":      10,
}
```

Find body IDs in Motive: **View → Assets → right-click body → Properties → ID**

Use these names everywhere in your code — `BODY_IDS["left_foot"]` — instead of raw integers.

### Step 4 — Set capture rate

```python
VELOCITY_RATE_HZ = 360
```

**This must match the frame rate configured in Motive.** If they differ, velocity will be computed at the wrong cadence.

---

## Setting the capture frequency

The NatNet publish rate equals the **camera capture frame rate** set in Motive.  
There is **no software ceiling** — the limit is your camera hardware.

### How to change it in Motive

1. Open Motive on the OptiTrack server PC
2. **Edit → Settings → Camera → Frame Rate**
3. Select the desired rate (e.g. 120, 240, 360 Hz)
4. Click **Apply** (takes effect immediately, no restart)
5. Run `frequency_probe.py` to confirm

### How to change it in the code

Open `config.py` and update one line:

```python
VELOCITY_RATE_HZ = 360   # ← change this to match Motive
```

That's it. Nothing else needs to change.

> **Rule:** `VELOCITY_RATE_HZ` in `config.py` should always equal the frame rate set in Motive.  
> Run `python examples/frequency_probe.py` after any change to verify they match.

### Camera hardware limits

| Camera series | Max frame rate |
|---|---|
| Prime 13 / 13W | 240 Hz |
| Prime 17W | 360 Hz |
| PrimeX 13 | 360 Hz |
| PrimeX 22 | 260 Hz |
| PrimeX 41 | 180 Hz |
| Slim 3U | 100 Hz |
| Flex 13 | 120 Hz |

Check your camera's spec sheet on the [OptiTrack website](https://optitrack.com/cameras/) for the exact limit.

### Buffer depth at high frame rates

At very high frame rates, increase `MAX_SAMPLES_PER_BODY` in `config.py`:

```python
# Rule: MAX_SAMPLES_PER_BODY  ≥  VELOCITY_WINDOW_S * VELOCITY_RATE_HZ * 2
# At 360 Hz: 0.040 * 360 * 2 = 28.8  →  60 gives comfortable headroom
# At 720 Hz: 0.040 * 720 * 2 = 57.6  →  increase to 80 or 128
MAX_SAMPLES_PER_BODY = 60
```

---

## Examples

```bash
# Print live pose (refreshes at 10 Hz)
python examples/read_body.py --name left_foot
python examples/read_body.py --id 101

# Measure actual NatNet publish rate
python examples/frequency_probe.py --name right_foot
```

The frequency probe auto-detects saturation and prints the recommended value for `VELOCITY_RATE_HZ`.

---

## Conversion utilities

```python
from mocap import quat_to_euler_xyz, quat_to_rotmat, euler_xyz_to_quat
import math

roll, pitch, yaw = quat_to_euler_xyz(quat)           # radians
roll_deg = math.degrees(roll)

R = quat_to_rotmat(quat)                              # 3×3 numpy array

quat = euler_xyz_to_quat(roll, pitch, yaw)
```

---

## NATNET_SDK_PATH override

To use a different NatNet SDK version:

```bash
export NATNET_SDK_PATH=/path/to/NatNet_SDK_x.x/samples/PythonClient
python examples/read_body.py
```

---

## Known issues and fixes (discovery log)

These bugs were found while running a 230 Hz client loop and are fixed in this client.

### Bug 1 — Pose update rate hard-capped at 50 Hz

**Symptom:** ~76% of control-loop reads returned the exact same position value (stale reads), even though the MoCap hardware was running at 120+ Hz.

**Root cause:** The original NatNet Python client only stored incoming frames in a rolling buffer. A separate background thread — running at a fixed 50 Hz — was the sole writer to the state cache that `get_body_state()` read from. So at a 230 Hz control loop rate, each new pose value was re-read ~4–5 times before the background thread ran again.

**Fix:** Position and orientation are now written into the state cache directly inside the NatNet frame callback, at the full hardware rate. The background thread still exists for velocity computation but is no longer the gatekeeper for pose data.

### Bug 2 — Velocity magnitude wrong at non-50 Hz rates

**Symptom:** After changing the background thread rate from 50 Hz to 360 Hz, linear velocity values were ~7× too large.

**Root cause:** The same variable was used as both the thread sleep interval and the denominator in the finite-difference formula:

```python
velocity = (avg_pos_2 - avg_pos_1) / self._velocity_dt   # WRONG
```

At 50 Hz, `_velocity_dt = 20 ms`. At 360 Hz, `_velocity_dt = 2.8 ms` — the denominator shrank by 7×, inflating the velocity.

**Fix:** The denominator is now a fixed constant (`VELOCITY_HALF_S = 20 ms` in `config.py`) completely independent of the loop rate. The two averaging half-windows are always `[now−40ms, now−20ms)` and `[now−20ms, now]`, regardless of how often they are computed.

### Getting above 120 Hz

The NatNet Python SDK and this client impose no upper limit on frame rate. The only limit is the Motive camera frame rate setting and the camera hardware. To go above 120 Hz: open Motive → **Edit → Settings → Camera → Frame Rate** → select a higher value → Apply. Then update `VELOCITY_RATE_HZ` in `config.py` to match.

---

## License

NatNet SDK files in `natnet_sdk/` are provided by NaturalPoint / OptiTrack under their [EULA](https://optitrack.com/software/natnet-sdk/).  
`mocap.py`, `config.py`, and examples are MIT licensed.
