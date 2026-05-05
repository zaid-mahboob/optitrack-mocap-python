"""
Measure the true NatNet publish frequency for a rigid body.

Use this to verify that Motive and the client are in sync after changing
VELOCITY_RATE_HZ in config.py or the frame rate in Motive.

Usage:
    python examples/frequency_probe.py                    # first body in config
    python examples/frequency_probe.py --id 101
    python examples/frequency_probe.py --name right_foot
"""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CLIENT_ADDRESS, SERVER_ADDRESS, USE_MULTICAST, BODY_IDS
from mocap import MocapClient

SATURATION_THRESHOLD_HZ = 1.0
WINDOWS = [1.0, 2.0, 4.0, 8.0, 16.0, 30.0]


class _ProbingClient(MocapClient):
    def __init__(self, body_id: int, **kwargs):
        super().__init__(**kwargs)
        self._probe_id = body_id
        self._times: list[float] = []
        self._times_lock = threading.Lock()

    def _on_new_frame(self, data_dict: dict) -> None:
        super()._on_new_frame(data_dict)
        mocap_data = data_dict.get("mocap_data")
        if mocap_data is None or mocap_data.rigid_body_data is None:
            return
        for rb in mocap_data.rigid_body_data.rigid_body_list:
            if rb.id_num == self._probe_id:
                with self._times_lock:
                    self._times.append(time.monotonic())
                break

    def snapshot(self) -> list[float]:
        with self._times_lock:
            return list(self._times)


def _stats(times, window_s):
    if len(times) < 2:
        return None
    w = [t for t in times if t >= times[-1] - window_s]
    if len(w) < 2:
        return None
    hz = 1.0 / np.diff(w)
    return dict(samples=len(w), mean=hz.mean(), median=float(np.median(hz)),
                std=hz.std(), min=hz.min(), max=hz.max())


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--id",   type=int)
    group.add_argument("--name", type=str)
    args = parser.parse_args()

    if args.name:
        body_id, label = BODY_IDS[args.name], args.name
    elif args.id:
        body_id, label = args.id, str(args.id)
    else:
        label, body_id = next(iter(BODY_IDS.items()))

    client = _ProbingClient(
        body_id=body_id,
        client_address=CLIENT_ADDRESS,
        server_address=SERVER_ADDRESS,
        use_multicast=USE_MULTICAST,
    )
    print(f"Connecting to Motive at {SERVER_ADDRESS} ...")
    if not client.start():
        print("ERROR: Could not connect.")
        sys.exit(1)
    print(f"Probing '{label}' (id={body_id}). Ctrl-C to stop early.\n")

    stop = threading.Event()
    prev = [0, time.monotonic()]

    def _live():
        while not stop.wait(1.0):
            times = client.snapshot()
            now   = time.monotonic()
            hz    = (len(times) - prev[0]) / (now - prev[1])
            span  = (times[-1] - times[0]) if len(times) >= 2 else 0.0
            print(f"\r  [live] frames={len(times):5d}  last 1s ≈ {hz:6.1f} Hz"
                  f"  collected {span:.1f}s", end="", flush=True)
            prev[0], prev[1] = len(times), now

    threading.Thread(target=_live, daemon=True).start()

    results = []
    try:
        for window in WINDOWS:
            while True:
                times = client.snapshot()
                if len(times) >= 2 and (times[-1] - times[0]) >= window + 1.0:
                    break
                time.sleep(0.1)

            s = _stats(client.snapshot(), window)
            if not s:
                continue
            results.append(s)

            if len(results) == 1:
                stop.set()
                time.sleep(0.1)
                print()
                print("─" * 70)
                print(f"  {'window':<8} {'samples':>8} {'mean':>9} {'median':>9}"
                      f" {'std':>7} {'min':>8} {'max':>8}")
                print("─" * 70)

            print(f"  {window:.0f}s       {s['samples']:>8d}"
                  f"  {s['mean']:>8.1f} Hz  {s['median']:>8.1f} Hz"
                  f"  {s['std']:>6.1f}  {s['min']:>7.1f}  {s['max']:>7.1f}")

            if len(results) >= 2:
                delta = abs(results[-1]["median"] - results[-2]["median"])
                if delta < SATURATION_THRESHOLD_HZ:
                    print("─" * 70)
                    print(f"\n  Saturated (Δmedian = {delta:.2f} Hz < {SATURATION_THRESHOLD_HZ} Hz)")
                    break
    except KeyboardInterrupt:
        stop.set()
        print()

    if results:
        b = results[-1]
        print(f"\n{'═'*70}")
        print(f"  NatNet rate for '{label}' (id={body_id})")
        print(f"  {b['median']:.1f} Hz median  ±{b['std']:.1f} Hz std  "
              f"over {b['samples']} frames")
        print(f"  → Set VELOCITY_RATE_HZ = {round(b['median'] / 60) * 60} in config.py")
        print(f"{'═'*70}")

    client.shutdown()


if __name__ == "__main__":
    main()
