# Camera Extrinsic Calibration: Complete Specification

## Overview

Two-phase procedure to determine the full 6-DOF transform from robot center (forward = +X, left = +Y, up = +Z) to camera frame, using a single AprilTag of unknown position and orientation, no physical alignment, no knowledge of world frame.

**Unknowns to solve:**
- `R_rc` — full 3D rotation of camera relative to robot (yaw, pitch, roll)
- `t_rc` — camera translation from robot CoR (x, y, z)

**Known / trusted:**
- Gyro gives reliable *delta* heading (Z rotation only)
- Swerve odometry gives reliable XY displacement in gyro frame
- AprilTag detector gives full 6-DOF `T_ct` (tag pose in camera frame) via PnP
- Robot moves in 2D (no pitch/roll of the robot body itself)

---

## Phase 1: Drive Forward → Camera Yaw

### Procedure

1. Place robot anywhere with tag clearly visible
2. Drive robot **straight forward** at least **1 meter** at slow constant speed
3. Record tag detections and gyro continuously

### Data Collected

```
detections_fwd = [(T_ct_i, ψ_i)]    # tag pose in camera frame, gyro heading
```

### Step 1a: Correct for Veering

Robot may not drive perfectly straight. Use gyro to rotate each tag observation back into a consistent frame:

```python
ψ_0 = gyro[0]
for each detection i:
    Δψ_i = ψ_i - ψ_0
    R_correction = R_z(-Δψ_i)
    p_tag_corrected_i = R_correction @ T_ct_i[:3, 3]
```

Now `p_tag_corrected_i` is the tag position as if the robot had driven perfectly straight.

### Step 1b: Fit Line to Tag XY Positions

```python
positions_xy = [p[:2] for p in p_tag_corrected]

_, _, Vt = svd(positions_xy - mean(positions_xy))
line_direction = Vt[0]                  # principal direction

# tag moves backward as robot moves forward — fix sign
if dot(line_direction, positions_xy[0] - positions_xy[-1]) < 0:
    line_direction = -line_direction

forward_in_cam_xy = normalize(line_direction)
```

### Step 1c: Extract Camera Yaw

```python
φ = atan2(forward_in_cam_xy[1], forward_in_cam_xy[0])
```

This is the only output of Phase 1. Pitch and roll are left to Phase 2.

---

## Phase 2: Spin in Place → Full Translation + Full Rotation

### Why This Works

Once camera yaw `φ` is known, we have a good enough initial `R_rc` to bootstrap. At each spin position, PnP gives the full camera pose in tag frame. Since the robot is only rotating (not translating), the camera orbits the CoR — the CoR is the one point whose world position is constant across all frames. Solving for `t_rc` is then a direct linear problem.

### Procedure

1. Keep robot at same XY position (tag still visible)
2. Spin robot in place **at least 180°**, ideally **360°**
3. Record tag detections and gyro continuously

### Data Collected

```
detections_spin = [(T_ct_i, ψ_i)]
```

### Step 2a: Get Camera Pose in Tag Frame via PnP

PnP (via the AprilTag detector) gives `T_ct_i` — the transform from camera to tag. Inverting gives camera position in tag frame:

```python
for each detection i:
    T_tc_i = inv(T_ct_i)               # camera pose in tag frame
    p_cam_tag_i = T_tc_i[:3, 3]        # camera position in tag frame
    R_cam_tag_i = T_tc_i[:3, :3]       # camera orientation in tag frame
```

### Step 2b: Solve for Camera Translation (t_rc)

The robot is spinning in place. The camera orbits the CoR. In world frame, the camera position at each step is:

```
p_cam_world_i = R_z(ψ_i) @ t_rc + p_robot_world
```

where `p_robot_world` is constant (robot didn't translate) and `t_rc` is the camera offset from CoR in robot frame.

We also know camera world position from PnP + the tag's (unknown but fixed) world pose:

```
p_cam_world_i = R_tag_world @ p_cam_tag_i + p_tag_world
```

Since `p_tag_world` and `p_robot_world` are both unknown constants, eliminate them by differencing pairs of frames:

```
(R_z(ψ_i) - R_z(ψ_j)) @ t_rc = R_tag_world @ (p_cam_tag_i - p_cam_tag_j)
```

Solve for `t_rc` and `R_tag_world` jointly via alternating least squares:

```python
Q = eye(3)      # initial guess for R_tag_world
for iteration in range(20):
    # Step A: solve for t_rc given Q
    A = []
    b = []
    for i in range(1, N):
        dR = R_z(ψ_i) - R_z(ψ_0)          # 3x3
        dp = p_cam_tag_i - p_cam_tag_0      # 3x1
        A.append(dR)
        b.append(Q @ dp)
    t_rc = lstsq(vstack(A), hstack(b))

    # Step B: solve for Q given t_rc (Procrustes)
    lhs = stack([R_z(ψ_i) @ t_rc - R_z(ψ_0) @ t_rc for i in range(1, N)])
    rhs = stack([p_cam_tag_i - p_cam_tag_0 for i in range(1, N)])
    U, _, Vt = svd(rhs.T @ lhs)
    Q = U @ Vt                              # nearest rotation matrix
```

This gives `t_rc` (all 3 components) without knowing where the tag or robot are in the world.

### Step 2c: Solve for Full Camera Rotation (R_rc)

Camera orientation in world frame at each step must equal robot orientation times camera-to-robot rotation:

```
R_cam_world_i = R_z(ψ_i) @ R_rc
```

We also have camera orientation in world from PnP:

```
R_cam_world_i = R_tag_world @ R_cam_tag_i
```

Setting equal and solving:

```
R_rc = R_z(ψ_i).T @ Q @ R_cam_tag_i
```

Average over all frames for robustness:

```python
R_rc_estimates = []
for i in range(N):
    R_rc_i = R_z(ψ_i).T @ Q @ R_cam_tag_i
    R_rc_estimates.append(R_rc_i)

# average rotation matrices via SVD
M = sum(R_rc_estimates)
U, _, Vt = svd(M)
R_rc = U @ Vt
```

---

## Phase 3: Joint Optimization

Use Phase 1 and 2 estimates as initial guess, refine everything jointly. The cost is consistency of the tag world pose across all measurements from both phases:

```python
x0 = [*t_rc, *rotation_to_rotvec(R_rc)]    # 6 DOF

def cost(params):
    t_rc = params[:3]
    R_rc = rotvec_to_rotation(params[3:])
    T_rc = make_transform(R_rc, t_rc)

    tag_world_estimates = []

    for (T_ct_i, ψ_i, x_i, y_i) in detections_fwd:
        T_wr_i = make_transform(R_z(ψ_i), [x_i, y_i, 0])
        T_wt_i = T_wr_i @ T_rc @ T_ct_i
        tag_world_estimates.append(T_wt_i)

    for (T_ct_i, ψ_i) in detections_spin:
        T_wr_i = make_transform(R_z(ψ_i), [x_spin, y_spin, 0])
        T_wt_i = T_wr_i @ T_rc @ T_ct_i
        tag_world_estimates.append(T_wt_i)

    # all estimates of tag world pose must agree
    mean_pos = mean([T[:3,3] for T in tag_world_estimates], axis=0)
    M = sum([T[:3,:3] for T in tag_world_estimates])
    U, _, Vt = svd(M)
    mean_rot = U @ Vt

    errors = []
    for T_wt in tag_world_estimates:
        errors.extend(T_wt[:3,3] - mean_pos)                  # position residual
        dR = T_wt[:3,:3] @ mean_rot.T
        errors.extend(rotation_to_rotvec(dR))                 # rotation residual

    return errors

result = least_squares(cost, x0)
T_rc_final = make_transform(
    rotvec_to_rotation(result.x[3:]),
    result.x[:3]
)
```

---

## Observability Summary

| Unknown | Phase 1 (forward) | Phase 2 (spin) |
|---------|:-----------------:|:--------------:|
| Camera yaw | ✅ Primary | ✅ Confirms |
| Camera pitch | ❌ | ✅ Via PnP |
| Camera roll | ❌ | ✅ Via PnP |
| tx (forward offset) | ❌ | ✅ Via circle fit |
| ty (lateral offset) | ❌ | ✅ Via circle fit |
| tz (height) | ❌ | ✅ Via circle fit |

Phase 1 exists solely to bootstrap camera yaw so Phase 2 has a good enough initial `R_rc` to work with. Phase 2 gives everything else.

---

## Data Quality Requirements

```
Phase 1 — Drive Forward:
  - Minimum travel distance:  1.0 m
  - Constant speed
  - Tag must remain visible entire time
  - Tag should be within ~45° of forward (not directly to the side)

Phase 2 — Spin in Place:
  - Minimum rotation:         180° (360° strongly preferred for full observability)
  - Constant angular velocity
  - Robot XY must not drift (swerve holds position)
  - Same tag visible entire time
  - Prefer large angular separation between frames when subsampling

General:
  - Apply tag detection confidence threshold before using any sample
  - Subsample to ~10 Hz to avoid autocorrelated noise dominating optimizer
  - Discard detections where tag is at extreme angle (degraded PnP accuracy)
```
