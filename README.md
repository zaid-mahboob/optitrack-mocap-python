# optitrack-python

Minimal Python client for the **OptiTrack NatNet** protocol. Returns full 6-DOF rigid body state — position, orientation, linear velocity, angular velocity — from any Motive server.

No ROS. No heavy dependencies. One config file controls everything.

---

## Quick start

**1. Edit `config.py`** — set your IPs, body IDs, and capture rate.

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
├── mocap.py              ← client library
├── natnet_sdk/           ← bundled NatNet 4.4 Python SDK
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

The NatNet SDK is pure Python and bundled in `natnet_sdk/`.

---

## Configuration

**`config.py` is the only file you need to edit.**

```python
CLIENT_ADDRESS = "169.254.160.50"   # IP of this machine on the MoCap network card
SERVER_ADDRESS = "169.254.160.46"   # IP of the machine running Motive

USE_MULTICAST = False   # False = unicast (one client); True = multicast (multiple clients)

BODY_IDS = {
    "left_foot":  100,
    "right_foot": 101,
    "object":      10,
}

VELOCITY_RATE_HZ = 360   # must match the frame rate set in Motive
```

`VELOCITY_RATE_HZ` must match the frame rate configured in Motive — run `frequency_probe.py` to confirm.

At very high frame rates, increase `MAX_SAMPLES_PER_BODY` to maintain buffer headroom:

```python
# Rule: MAX_SAMPLES_PER_BODY  ≥  VELOCITY_WINDOW_S * VELOCITY_RATE_HZ * 2
MAX_SAMPLES_PER_BODY = 60
```

---

## Examples

```bash
# Print live pose
python examples/read_body.py --name left_foot
python examples/read_body.py --id 101

# Measure actual NatNet publish rate
python examples/frequency_probe.py --name right_foot
```

---

## Conversion utilities

```python
from mocap import quat_to_euler_xyz, quat_to_rotmat, euler_xyz_to_quat
import math

roll, pitch, yaw = quat_to_euler_xyz(quat)   # radians
R = quat_to_rotmat(quat)                      # 3×3 numpy array
quat = euler_xyz_to_quat(roll, pitch, yaw)
```

---

## License

NatNet SDK files in `natnet_sdk/` are provided by NaturalPoint / OptiTrack under their [EULA](https://optitrack.com/software/natnet-sdk/).  
`mocap.py`, `config.py`, and examples are MIT licensed.
