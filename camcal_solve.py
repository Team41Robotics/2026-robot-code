#!/usr/bin/env python3
"""
Camera extrinsic calibration solver for FRC robot.

Reads CSV data collected by CameraCalibration.java and solves for the
6-DOF transform from robot center to camera (T_rc).

Since multi-tag PnP gives camera-to-field-origin (known reference frame),
the unknown Q matrix from the general spec collapses to identity, allowing
a direct linear solve in Phase 2.

Usage: python camcal_solve.py camcal_data.csv
"""

import sys
import csv
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def load_csv(path):
    samples = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            samples.append({
                "phase": row["phase"],
                "timestamp": float(row["timestamp"]),
                "imuYaw": float(row["imuYaw"]),
                "odomX": float(row["odomX"]),
                "odomY": float(row["odomY"]),
                "tx": float(row["tx"]),
                "ty": float(row["ty"]),
                "tz": float(row["tz"]),
                "qw": float(row["qw"]),
                "qx": float(row["qx"]),
                "qy": float(row["qy"]),
                "qz": float(row["qz"]),
            })
    return samples


def quat_to_rot(qw, qx, qy, qz):
    return Rotation.from_quat([qx, qy, qz, qw]).as_matrix()


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotvec_to_rot(rv):
    return Rotation.from_rotvec(rv).as_matrix()


def rot_to_rotvec(R):
    return Rotation.from_matrix(R).as_rotvec()


def nearest_rotation(M):
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def sample_to_T_ct(s):
    R = quat_to_rot(s["qw"], s["qx"], s["qy"], s["qz"])
    t = np.array([s["tx"], s["ty"], s["tz"]])
    return make_transform(R, t)


# ---------------------------------------------------------------------------
# Phase 1: Drive forward -> camera yaw
# ---------------------------------------------------------------------------
def phase1_camera_yaw(fwd_samples):
    if len(fwd_samples) < 3:
        raise ValueError(f"Need >= 3 forward samples, got {len(fwd_samples)}")

    psi_0 = fwd_samples[0]["imuYaw"]

    # Correct tag (field-origin) positions in camera frame for veering
    corrected = []
    for s in fwd_samples:
        dpsi = s["imuYaw"] - psi_0
        p_cam = np.array([s["tx"], s["ty"], s["tz"]])
        corrected.append(Rz(-dpsi) @ p_cam)
    corrected = np.array(corrected)

    # SVD line fit on XY
    xy = corrected[:, :2]
    _, _, Vt = np.linalg.svd(xy - np.mean(xy, axis=0))
    direction = Vt[0]

    # Tag moves backward as robot moves forward
    if np.dot(direction, xy[0] - xy[-1]) < 0:
        direction = -direction

    phi = np.arctan2(direction[1], direction[0])
    print(f"[Phase 1] Camera yaw: {np.degrees(phi):.2f} deg")
    print(f"  Samples: {len(fwd_samples)}")
    print(f"  Line direction: [{direction[0]:.4f}, {direction[1]:.4f}]")
    return phi


# ---------------------------------------------------------------------------
# Phase 2: Spin in place -> full t_rc and R_rc
# ---------------------------------------------------------------------------
def phase2_solve(spin_samples, phi):
    if len(spin_samples) < 5:
        raise ValueError(f"Need >= 5 spin samples, got {len(spin_samples)}")

    # Camera poses in field frame (inv of camera-to-field)
    p_cam_field = []
    R_cam_field = []
    psi = []
    for s in spin_samples:
        T_fc = inv_transform(sample_to_T_ct(s))
        p_cam_field.append(T_fc[:3, 3])
        R_cam_field.append(T_fc[:3, :3])
        psi.append(s["imuYaw"])

    p_cam_field = np.array(p_cam_field)
    psi = np.array(psi)

    # --- Solve t_rc (XY) via direct linear system ---
    # (Rz(psi_i) - Rz(psi_0)) @ t_rc = p_cam_field_i - p_cam_field_0
    # Z rows of Rz difference are zero, so only XY is constrained here
    A_rows = []
    b_rows = []
    for i in range(1, len(spin_samples)):
        dR = Rz(psi[i]) - Rz(psi[0])
        dp = p_cam_field[i] - p_cam_field[0]
        A_rows.append(dR[:2, :2])  # 2x2, only affects tx/ty
        b_rows.append(dp[:2])

    A = np.vstack(A_rows)
    b = np.concatenate(b_rows)
    t_rc_xy, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)

    # Z: camera height above robot center = camera height in field frame
    # (robot center is at floor level, field z=0 is floor)
    t_rc_z = np.mean(p_cam_field[:, 2])
    t_rc = np.array([t_rc_xy[0], t_rc_xy[1], t_rc_z])

    print(f"[Phase 2] t_rc: [{t_rc[0]:.4f}, {t_rc[1]:.4f}, {t_rc[2]:.4f}] m")

    # --- Solve R_rc ---
    # R_cam_field_i = Rz(psi_i) @ R_rc  =>  R_rc = Rz(psi_i)^T @ R_cam_field_i
    R_sum = np.zeros((3, 3))
    for i in range(len(spin_samples)):
        R_sum += Rz(psi[i]).T @ R_cam_field[i]
    R_rc = nearest_rotation(R_sum)

    ypr = Rotation.from_matrix(R_rc).as_euler("ZYX", degrees=True)
    print(f"[Phase 2] R_rc (yaw, pitch, roll): [{ypr[0]:.2f}, {ypr[1]:.2f}, {ypr[2]:.2f}] deg")
    print(f"  Samples: {len(spin_samples)}")
    if len(residuals) > 0:
        print(f"  XY residual (RMS): {np.sqrt(residuals.sum() / len(b)):.6f} m")

    return t_rc, R_rc


# ---------------------------------------------------------------------------
# Phase 3: Joint optimization over all data
# ---------------------------------------------------------------------------
def phase3_optimize(all_samples, t_rc_init, R_rc_init):
    rv_init = rot_to_rotvec(R_rc_init)
    x0 = np.concatenate([t_rc_init, rv_init])

    def cost(params):
        t_rc = params[:3]
        R_rc = rotvec_to_rot(params[3:])
        T_rc = make_transform(R_rc, t_rc)

        # For each sample, compute T_odom_robot @ T_rc @ T_ct
        # All should equal T_odom_field (a single unknown constant)
        positions = []
        rotations = []
        for s in all_samples:
            T_ct = sample_to_T_ct(s)
            T_wr = make_transform(Rz(s["imuYaw"]), [s["odomX"], s["odomY"], 0.0])
            T_wt = T_wr @ T_rc @ T_ct
            positions.append(T_wt[:3, 3])
            rotations.append(T_wt[:3, :3])

        positions = np.array(positions)
        mean_pos = np.mean(positions, axis=0)
        mean_rot = nearest_rotation(sum(rotations))

        errors = []
        for i in range(len(all_samples)):
            errors.extend(positions[i] - mean_pos)
            dR = rotations[i] @ mean_rot.T
            errors.extend(rot_to_rotvec(dR))
        return np.array(errors)

    result = least_squares(cost, x0, method="lm")
    t_rc = result.x[:3]
    R_rc = rotvec_to_rot(result.x[3:])

    ypr = Rotation.from_matrix(R_rc).as_euler("ZYX", degrees=True)
    print(f"\n[Phase 3] Optimized t_rc: [{t_rc[0]:.4f}, {t_rc[1]:.4f}, {t_rc[2]:.4f}] m")
    print(f"[Phase 3] Optimized R_rc (yaw, pitch, roll): [{ypr[0]:.2f}, {ypr[1]:.2f}, {ypr[2]:.2f}] deg")
    print(f"  Cost: {result.cost:.6f}, iterations: {result.nfev}")
    return t_rc, R_rc


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def print_wpilib(t_rc, R_rc):
    # WPILib Rotation3d(roll, pitch, yaw) in radians
    ypr = Rotation.from_matrix(R_rc).as_euler("ZYX")  # [yaw, pitch, roll] rad
    print()
    print("=" * 60)
    print("Copy into Vision.java cameras[]:")
    print("=" * 60)
    print(f"new Transform3d(")
    print(f"    new Translation3d({t_rc[0]:.6f}, {t_rc[1]:.6f}, {t_rc[2]:.6f}),")
    print(f"    new Rotation3d({ypr[2]:.6f}, {ypr[1]:.6f}, {ypr[0]:.6f}))")
    print("=" * 60)
    ypr_deg = np.degrees(ypr)
    print(f"\n  X (forward): {t_rc[0]:+.4f} m")
    print(f"  Y (left):    {t_rc[1]:+.4f} m")
    print(f"  Z (up):      {t_rc[2]:+.4f} m")
    print(f"  Yaw:         {ypr_deg[0]:+.2f} deg")
    print(f"  Pitch:       {ypr_deg[1]:+.2f} deg")
    print(f"  Roll:        {ypr_deg[2]:+.2f} deg")


def main():
    if len(sys.argv) < 2:
        print("Usage: python camcal_solve.py <camcal_data.csv>")
        sys.exit(1)

    samples = load_csv(sys.argv[1])
    fwd = [s for s in samples if s["phase"] == "FORWARD"]
    spin = [s for s in samples if s["phase"] == "SPIN"]
    print(f"Loaded {len(samples)} samples  (FORWARD: {len(fwd)}, SPIN: {len(spin)})\n")

    phi = phase1_camera_yaw(fwd)
    print()
    t_rc, R_rc = phase2_solve(spin, phi)
    t_rc, R_rc = phase3_optimize(fwd + spin, t_rc, R_rc)
    print_wpilib(t_rc, R_rc)


if __name__ == "__main__":
    main()
