# =============================================================================
# OptiTrack MoCap Configuration
# Change everything here. Nothing else needs to be edited.
# =============================================================================

# -----------------------------------------------------------------------------
# Network
# -----------------------------------------------------------------------------

# IP address of THIS machine on the MoCap network interface.
# Find it with: ip addr   (look for the NIC connected to the OptiTrack switch)
CLIENT_ADDRESS = "169.254.160.50"

# IP address of the machine running Motive.
# Find it in Motive: Edit > Settings > Network > Local Interface
SERVER_ADDRESS = "169.254.160.46"

# Streaming mode.
# False = unicast  (simpler, point-to-point, recommended for single client)
# True  = multicast (needed when multiple machines receive the same stream)
# Must match the setting in Motive: Edit > Settings > Streaming > Transmission Type
USE_MULTICAST = False


# -----------------------------------------------------------------------------
# Rigid body IDs
# Give your bodies meaningful names here. Use these names in your code instead
# of raw integers so it's clear what each ID refers to.
#
# Find body IDs in Motive: View > Assets > (right-click body) > Properties > ID
# -----------------------------------------------------------------------------
BODY_IDS = {
    "left_foot":  100,
    "right_foot": 101,
    "object":      10,
}


# -----------------------------------------------------------------------------
# Capture rate
#
# Set this to match the frame rate configured in Motive.
# To change the capture rate in Motive:
#   Edit > Settings > Camera > Frame Rate  →  set to desired Hz  →  Apply
#
# There is NO upper limit enforced in software. The ceiling is your camera
# hardware. Common OptiTrack cameras and their max rates:
#   Prime 13/13W         → 240 Hz
#   Prime 17W            → 360 Hz
#   Prime 41             → 180 Hz
#   Slim 3U              → 100 Hz
#   PrimeX 13 / 22 / 41 → up to 360 Hz (model dependent)
#
# This value controls how often the velocity background thread recomputes.
# It should match (or be close to) your Motive frame rate for minimum latency.
# -----------------------------------------------------------------------------
VELOCITY_RATE_HZ = 360


# -----------------------------------------------------------------------------
# Velocity computation window  (advanced — only change if you know why)
#
# Velocity is computed as a finite difference over a two-half look-back window:
#
#   |<-------- VELOCITY_WINDOW_S = 40 ms -------->|
#   |<-- half 1: 40ms→20ms -->|<-- half 2: 20ms→now -->|
#
#   linear_velocity  ≈ (mean_pos(half2)  - mean_pos(half1))  / VELOCITY_HALF_S
#   angular_velocity ≈ quat_diff(half2, half1) / VELOCITY_HALF_S
#
# VELOCITY_HALF_S must equal VELOCITY_WINDOW_S / 2.
# Increasing VELOCITY_WINDOW_S smooths velocity but adds latency.
# -----------------------------------------------------------------------------
VELOCITY_WINDOW_S    = 0.040   # 40 ms total look-back
VELOCITY_HALF_S      = 0.020   # 20 ms half-window denominator


# -----------------------------------------------------------------------------
# Frame buffer depth  (advanced)
#
# Number of frames kept in the rolling buffer per body for velocity computation.
# Must cover at least:  VELOCITY_WINDOW_S * VELOCITY_RATE_HZ  frames.
#
#   Example at 360 Hz: 0.040 * 360 = 14.4  →  60 gives 4× headroom.
#   Example at 960 Hz: 0.040 * 960 = 38.4  →  increase to 80 or 128.
# -----------------------------------------------------------------------------
MAX_SAMPLES_PER_BODY = 60
