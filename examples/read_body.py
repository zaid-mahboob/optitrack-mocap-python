"""
Print live pose of a rigid body.  All settings come from config.py.

Usage:
    python examples/read_body.py                    # first body in config BODY_IDS
    python examples/read_body.py --id 101           # by numeric ID
    python examples/read_body.py --name right_foot  # by name from config
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CLIENT_ADDRESS, SERVER_ADDRESS, USE_MULTICAST, BODY_IDS
from mocap import MocapClient, quat_to_euler_xyz


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--id",   type=int, help="Rigid body ID (integer)")
    group.add_argument("--name", type=str, help="Rigid body name from config BODY_IDS")
    args = parser.parse_args()

    if args.name:
        if args.name not in BODY_IDS:
            print(f"ERROR: '{args.name}' not in BODY_IDS. Available: {list(BODY_IDS)}")
            sys.exit(1)
        body_id = BODY_IDS[args.name]
        label   = args.name
    elif args.id:
        body_id = args.id
        label   = str(args.id)
    else:
        label, body_id = next(iter(BODY_IDS.items()))

    client = MocapClient(client_address=CLIENT_ADDRESS,
                         server_address=SERVER_ADDRESS,
                         use_multicast=USE_MULTICAST)
    print(f"Connecting to Motive at {SERVER_ADDRESS} ...")
    if not client.start():
        print("ERROR: Could not connect.")
        sys.exit(1)
    print(f"Streaming '{label}' (id={body_id}). Ctrl-C to stop.\n")

    try:
        while True:
            state = client.get_body_state(body_id)
            print("\033[H\033[J", end="")
            if state is None:
                print(f"{label} (id={body_id}): not visible")
            else:
                pos, quat, linvel, angvel = state
                r, p, y = quat_to_euler_xyz(quat)
                print(f"{label}  (id={body_id})")
                print(f"  pos      x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}  m")
                print(f"  rpy      r={math.degrees(r):+.1f}  p={math.degrees(p):+.1f}"
                      f"  y={math.degrees(y):+.1f}  deg")
                print(f"  linvel   x={linvel[0]:+.3f}  y={linvel[1]:+.3f}  z={linvel[2]:+.3f}  m/s")
                print(f"  angvel   x={angvel[0]:+.3f}  y={angvel[1]:+.3f}  z={angvel[2]:+.3f}  rad/s")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.shutdown()
        print("\nDone.")


if __name__ == "__main__":
    main()
