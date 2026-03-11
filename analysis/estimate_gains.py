"""Estimate swerve drive/turn FF gains (kS, kV, kA) and evaluate PID from match logs.

Uses ordinary least squares on: V_motor = kS*sign(v) + kV*v + kA*a
Also analyzes PID error response to suggest kP adjustments.
"""
import sys, math, os
from collections import defaultdict
from wpiutil.log import DataLogReader
import numpy as np

# ---------------------------------------------------------------------------
# Generic readers (same as other analysis scripts)
# ---------------------------------------------------------------------------
def read_doubles(reader, keys):
    entries = {}
    data = defaultdict(list)
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in keys:
                entries[d.entry] = d.name
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entries:
                try:
                    val = record.getDouble()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

def read_booleans(reader, keys):
    entries = {}
    data = defaultdict(list)
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in keys:
                entries[d.entry] = d.name
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entries:
                try:
                    val = record.getBoolean()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

def find_enabled_periods(enabled_series):
    periods = []
    start = None
    for t, v in enabled_series:
        if v and start is None:
            start = t
        elif not v and start is not None:
            periods.append((start, t))
            start = None
    if start is not None:
        periods.append((start, enabled_series[-1][0]))
    return periods

def filter_enabled(series, enabled_periods):
    out = []
    for s, e in enabled_periods:
        out.extend([(t, v) for t, v in series if s <= t <= e])
    return out

def to_arrays(series):
    """Convert [(t,v),...] to (times, values) numpy arrays."""
    if not series:
        return np.array([]), np.array([])
    t = np.array([x[0] for x in series])
    v = np.array([x[1] for x in series])
    return t, v

def align_by_time(series_a, series_b, tol=0.015):
    """Align two time series by nearest timestamp within tolerance.
    Returns (times, vals_a, vals_b) arrays."""
    if not series_a or not series_b:
        return np.array([]), np.array([]), np.array([])
    ta, va = to_arrays(series_a)
    tb, vb = to_arrays(series_b)
    times, a_out, b_out = [], [], []
    j = 0
    for i in range(len(ta)):
        while j < len(tb) - 1 and tb[j+1] <= ta[i]:
            j += 1
        if j < len(tb) and abs(tb[j] - ta[i]) < tol:
            times.append(ta[i])
            a_out.append(va[i])
            b_out.append(vb[j])
    return np.array(times), np.array(a_out), np.array(b_out)

def numerical_derivative(times, values):
    """Compute numerical derivative (central difference where possible)."""
    n = len(times)
    if n < 3:
        return times, np.zeros(n)
    deriv = np.zeros(n)
    for i in range(1, n-1):
        dt = times[i+1] - times[i-1]
        if dt > 0:
            deriv[i] = (values[i+1] - values[i-1]) / dt
    deriv[0] = deriv[1]
    deriv[-1] = deriv[-2]
    return times, deriv

# ---------------------------------------------------------------------------
# FF estimation via OLS: V = kS*sign(v) + kV*v + kA*a
# ---------------------------------------------------------------------------
def estimate_ff(voltage_series, velocity_series, enabled_periods, label=""):
    """Estimate kS, kV, kA from motor voltage and velocity data."""
    volt_en = filter_enabled(voltage_series, enabled_periods)
    vel_en = filter_enabled(velocity_series, enabled_periods)

    t, volt, vel = align_by_time(volt_en, vel_en)
    if len(t) < 50:
        print(f"  {label}: insufficient data ({len(t)} samples)")
        return None

    # Compute acceleration from velocity
    _, accel = numerical_derivative(t, vel)

    # Filter: only use samples where robot is actually moving (|v| > 0.1)
    # and voltage isn't saturated
    mask = (np.abs(vel) > 0.1) & (np.abs(volt) < 11.5)
    t_f = t[mask]
    volt_f = volt[mask]
    vel_f = vel[mask]
    accel_f = accel[mask]

    if len(vel_f) < 30:
        print(f"  {label}: insufficient moving samples ({len(vel_f)} after filter)")
        return None

    # Build regression matrix: V = kS*sign(v) + kV*v + kA*a
    A = np.column_stack([np.sign(vel_f), vel_f, accel_f])
    # OLS solve
    result, residuals, rank, sv = np.linalg.lstsq(A, volt_f, rcond=None)
    kS, kV, kA = result

    # Compute R^2
    predicted = A @ result
    ss_res = np.sum((volt_f - predicted) ** 2)
    ss_tot = np.sum((volt_f - np.mean(volt_f)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

    # RMS error
    rms = np.sqrt(np.mean((volt_f - predicted) ** 2))

    print(f"  {label}:")
    print(f"    kS = {kS:.5f}  kV = {kV:.5f}  kA = {kA:.5f}")
    print(f"    R² = {r2:.4f}  RMS error = {rms:.3f}V  ({len(vel_f)} samples)")

    return {"kS": kS, "kV": kV, "kA": kA, "r2": r2, "rms": rms, "n": len(vel_f)}

# ---------------------------------------------------------------------------
# Turn FF estimation: same approach but for angular velocity
# ---------------------------------------------------------------------------
def estimate_turn_ff(voltage_series, velocity_series, enabled_periods, label=""):
    """Estimate turn kS, kV, kA from motor voltage and angular velocity."""
    volt_en = filter_enabled(voltage_series, enabled_periods)
    vel_en = filter_enabled(velocity_series, enabled_periods)

    t, volt, vel = align_by_time(volt_en, vel_en)
    if len(t) < 50:
        print(f"  {label}: insufficient data ({len(t)} samples)")
        return None

    _, accel = numerical_derivative(t, vel)

    # Filter: turning (|w| > 0.05 rad/s) and not saturated
    mask = (np.abs(vel) > 0.05) & (np.abs(volt) < 11.5)
    vel_f = vel[mask]
    volt_f = volt[mask]
    accel_f = accel[mask]

    if len(vel_f) < 30:
        print(f"  {label}: insufficient turning samples ({len(vel_f)} after filter)")
        return None

    A = np.column_stack([np.sign(vel_f), vel_f, accel_f])
    result, _, _, _ = np.linalg.lstsq(A, volt_f, rcond=None)
    kS, kV, kA = result

    predicted = A @ result
    ss_res = np.sum((volt_f - predicted) ** 2)
    ss_tot = np.sum((volt_f - np.mean(volt_f)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
    rms = np.sqrt(np.mean((volt_f - predicted) ** 2))

    print(f"  {label}:")
    print(f"    kS = {kS:.5f}  kV = {kV:.5f}  kA = {kA:.5f}")
    print(f"    R² = {r2:.4f}  RMS error = {rms:.3f}V  ({len(vel_f)} samples)")

    return {"kS": kS, "kV": kV, "kA": kA, "r2": r2, "rms": rms, "n": len(vel_f)}

# ---------------------------------------------------------------------------
# PID analysis: look at error dynamics to evaluate kP effectiveness
# ---------------------------------------------------------------------------
def analyze_pid(error_series, voltage_series, ff_series, velocity_series,
                enabled_periods, label="", is_drive=True):
    """Analyze PID performance from error, voltage, and FF data."""
    err_en = filter_enabled(error_series, enabled_periods)
    volt_en = filter_enabled(voltage_series, enabled_periods)
    ff_en = filter_enabled(ff_series, enabled_periods)
    vel_en = filter_enabled(velocity_series, enabled_periods)

    if not err_en:
        return

    t_err, err = to_arrays(err_en)
    abs_err = np.abs(err)

    print(f"\n  {label} PID analysis:")
    print(f"    Error: mean={np.mean(abs_err):.4f}  p50={np.median(abs_err):.4f}  "
          f"p95={np.percentile(abs_err, 95):.4f}  max={np.max(abs_err):.4f}")

    if is_drive:
        unit = "m/s"
    else:
        unit = f"rad ({math.degrees(np.mean(abs_err)):.2f}deg mean)"
    print(f"    Units: {unit}")

    # Estimate what kP is doing: PID_voltage = total_voltage - FF_voltage
    t_v, volt, ff = align_by_time(volt_en, ff_en)
    if len(t_v) > 50:
        pid_voltage = volt - ff
        # Align with error
        err_dict = {}
        for te, ve in err_en:
            err_dict[round(te, 2)] = ve
        pid_err_pairs = []
        for i, ti in enumerate(t_v):
            e = err_dict.get(round(ti, 2))
            if e is not None and abs(e) > 0.001:
                pid_err_pairs.append((e, pid_voltage[i]))

        if len(pid_err_pairs) > 30:
            errs = np.array([p[0] for p in pid_err_pairs])
            pids = np.array([p[1] for p in pid_err_pairs])
            # Simple linear fit: PID_voltage ≈ -kP_effective * error
            # (negative because error = measured - target, PID corrects opposite)
            A = np.column_stack([errs])
            result, _, _, _ = np.linalg.lstsq(A, pids, rcond=None)
            kP_eff = -result[0]  # negate because error convention
            print(f"    Effective kP from data: {kP_eff:.3f}")

    # Analyze error distribution during high-speed driving
    if is_drive and vel_en:
        t_v2, vel2, err2 = align_by_time(vel_en, err_en)
        if len(t_v2) > 50:
            high_speed = np.abs(vel2) > 1.0
            if np.sum(high_speed) > 20:
                hs_err = np.abs(err2[high_speed])
                print(f"    High-speed (>1m/s) error: mean={np.mean(hs_err):.4f}  "
                      f"p95={np.percentile(hs_err, 95):.4f}")
            low_speed = (np.abs(vel2) > 0.1) & (np.abs(vel2) <= 1.0)
            if np.sum(low_speed) > 20:
                ls_err = np.abs(err2[low_speed])
                print(f"    Low-speed (0.1-1m/s) error: mean={np.mean(ls_err):.4f}  "
                      f"p95={np.percentile(ls_err, 95):.4f}")

    # Check for oscillation: zero-crossings of error
    sign_changes = np.sum(np.abs(np.diff(np.sign(err[np.abs(err) > 0.01]))))
    duration = t_err[-1] - t_err[0] if len(t_err) > 1 else 1
    osc_freq = sign_changes / (2 * duration)  # oscillations per second
    print(f"    Error oscillation rate: {osc_freq:.1f} Hz")
    if osc_freq > 10:
        print(f"    ** HIGH OSCILLATION — kP may be too high or kD needed **")

# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------
def analyze_file(path):
    print(f"\n{'='*90}")
    print(f"  FF/PID GAIN ESTIMATION: {os.path.basename(path)}")
    print(f"{'='*90}")

    # Read enabled state
    reader = DataLogReader(path)
    enabled_data = read_booleans(reader, {"/DriverStation/Enabled"})
    enabled = enabled_data.get("/DriverStation/Enabled", [])
    enabled_periods = find_enabled_periods(enabled)
    total_enabled = sum(e - s for s, e in enabled_periods)
    print(f"\n  Enabled time: {total_enabled:.1f}s across {len(enabled_periods)} periods")

    if total_enabled < 5:
        print("  Too little enabled time for analysis.")
        return

    # Read all swerve data
    keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        keys.add(f"/Swerve/{corner}/DriveVoltageVolts")
        keys.add(f"/Swerve/{corner}/DriveVelMetersPerSec")
        keys.add(f"/Swerve/{corner}/DrivePosMeters")
        keys.add(f"/Swerve/{corner}/TurnVoltageVolts")
        keys.add(f"/Swerve/{corner}/TurnVelRadiansPerSec")
        keys.add(f"/Swerve/{corner}/TurnPosRadians")
        keys.add(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{corner}/turnErrorRadians")
        keys.add(f"/RealOutputs/Swerve/{corner}/driveFFVolts")
        keys.add(f"/RealOutputs/Swerve/{corner}/turnFFVolts")
        keys.add(f"/RealOutputs/Swerve/{corner}/setpointVelMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{corner}/setpointAngVelRadiansPerSec")

    reader = DataLogReader(path)
    data = read_doubles(reader, keys)

    # ========================
    # DRIVE FF ESTIMATION
    # ========================
    print(f"\n{'-'*60}")
    print(f"  DRIVE FEEDFORWARD  (current: kS={0.093052}, kV={1.8968}, kA={0.15096})")
    print(f"{'-'*60}")

    all_drive_results = []
    for corner in ["NE", "NW", "SE", "SW"]:
        volt = data.get(f"/Swerve/{corner}/DriveVoltageVolts", [])
        vel = data.get(f"/Swerve/{corner}/DriveVelMetersPerSec", [])
        result = estimate_ff(volt, vel, enabled_periods, label=corner)
        if result:
            all_drive_results.append(result)

    if all_drive_results:
        # Average across modules
        avg_kS = np.mean([r["kS"] for r in all_drive_results])
        avg_kV = np.mean([r["kV"] for r in all_drive_results])
        avg_kA = np.mean([r["kA"] for r in all_drive_results])
        avg_r2 = np.mean([r["r2"] for r in all_drive_results])

        std_kS = np.std([r["kS"] for r in all_drive_results])
        std_kV = np.std([r["kV"] for r in all_drive_results])
        std_kA = np.std([r["kA"] for r in all_drive_results])

        print(f"\n  >>> DRIVE FF AVERAGE (all modules):")
        print(f"      kS = {avg_kS:.5f} ± {std_kS:.5f}  (code: 0.093052)")
        print(f"      kV = {avg_kV:.5f} ± {std_kV:.5f}  (code: 1.8968)")
        print(f"      kA = {avg_kA:.5f} ± {std_kA:.5f}  (code: 0.15096)")
        print(f"      R² = {avg_r2:.4f}")

        # Check for module-to-module variation
        if std_kV / abs(avg_kV) > 0.1:
            print(f"  ** WARNING: >10% kV variation between modules — check for mechanical differences **")

    # ========================
    # TURN FF ESTIMATION
    # ========================
    print(f"\n{'-'*60}")
    print(f"  TURN FEEDFORWARD  (current: kS={0.19431}, kV={0.36606}, kA={0.0})")
    print(f"{'-'*60}")

    all_turn_results = []
    for corner in ["NE", "NW", "SE", "SW"]:
        volt = data.get(f"/Swerve/{corner}/TurnVoltageVolts", [])
        vel = data.get(f"/Swerve/{corner}/TurnVelRadiansPerSec", [])
        result = estimate_turn_ff(volt, vel, enabled_periods, label=corner)
        if result:
            all_turn_results.append(result)

    if all_turn_results:
        avg_kS = np.mean([r["kS"] for r in all_turn_results])
        avg_kV = np.mean([r["kV"] for r in all_turn_results])
        avg_kA = np.mean([r["kA"] for r in all_turn_results])
        avg_r2 = np.mean([r["r2"] for r in all_turn_results])

        std_kS = np.std([r["kS"] for r in all_turn_results])
        std_kV = np.std([r["kV"] for r in all_turn_results])
        std_kA = np.std([r["kA"] for r in all_turn_results])

        print(f"\n  >>> TURN FF AVERAGE (all modules):")
        print(f"      kS = {avg_kS:.5f} ± {std_kS:.5f}  (code: 0.19431)")
        print(f"      kV = {avg_kV:.5f} ± {std_kV:.5f}  (code: 0.36606)")
        print(f"      kA = {avg_kA:.5f} ± {std_kA:.5f}  (code: 0.0)")
        print(f"      R² = {avg_r2:.4f}")

    # ========================
    # PID ANALYSIS
    # ========================
    print(f"\n{'-'*60}")
    print(f"  DRIVE PID ANALYSIS  (current: kP={4})")
    print(f"{'-'*60}")

    for corner in ["NE", "NW", "SE", "SW"]:
        err = data.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        volt = data.get(f"/Swerve/{corner}/DriveVoltageVolts", [])
        ff = data.get(f"/RealOutputs/Swerve/{corner}/driveFFVolts", [])
        vel = data.get(f"/Swerve/{corner}/DriveVelMetersPerSec", [])
        if err:
            analyze_pid(err, volt, ff, vel, enabled_periods, label=corner, is_drive=True)

    print(f"\n{'-'*60}")
    print(f"  TURN PID ANALYSIS  (current: kP={20}, kD={0.4})")
    print(f"{'-'*60}")

    for corner in ["NE", "NW", "SE", "SW"]:
        err = data.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])
        volt = data.get(f"/Swerve/{corner}/TurnVoltageVolts", [])
        ff = data.get(f"/RealOutputs/Swerve/{corner}/turnFFVolts", [])
        vel = data.get(f"/Swerve/{corner}/TurnVelRadiansPerSec", [])
        if err:
            analyze_pid(err, volt, ff, vel, enabled_periods, label=corner, is_drive=False)

    # ========================
    # SUMMARY & RECOMMENDATIONS
    # ========================
    print(f"\n{'-'*60}")
    print(f"  RECOMMENDATIONS")
    print(f"{'-'*60}")

    if all_drive_results:
        avg_kS = np.mean([r["kS"] for r in all_drive_results])
        avg_kV = np.mean([r["kV"] for r in all_drive_results])
        avg_kA = np.mean([r["kA"] for r in all_drive_results])
        avg_r2 = np.mean([r["r2"] for r in all_drive_results])

        if avg_r2 > 0.8:
            print(f"\n  Drive FF (high confidence, R²={avg_r2:.3f}):")
            print(f"    DRIVE_kS = {abs(avg_kS):.5f};")
            print(f"    DRIVE_kV = {avg_kV:.5f};")
            print(f"    DRIVE_kA = {avg_kA:.5f};")
        else:
            print(f"\n  Drive FF (low confidence, R²={avg_r2:.3f} — use with caution):")
            print(f"    DRIVE_kS = {abs(avg_kS):.5f};")
            print(f"    DRIVE_kV = {avg_kV:.5f};")
            print(f"    DRIVE_kA = {avg_kA:.5f};")

    if all_turn_results:
        avg_kS = np.mean([r["kS"] for r in all_turn_results])
        avg_kV = np.mean([r["kV"] for r in all_turn_results])
        avg_kA = np.mean([r["kA"] for r in all_turn_results])
        avg_r2 = np.mean([r["r2"] for r in all_turn_results])

        if avg_r2 > 0.8:
            print(f"\n  Turn FF (high confidence, R²={avg_r2:.3f}):")
        else:
            print(f"\n  Turn FF (low confidence, R²={avg_r2:.3f} — use with caution):")
        print(f"    TURN_kS = {abs(avg_kS):.5f};")
        print(f"    TURN_kV = {avg_kV:.5f};")
        print(f"    TURN_kA = {avg_kA:.5f};")

    print()


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../logs"

    if os.path.isfile(log_dir):
        analyze_file(log_dir)
        return

    files = sorted([f for f in os.listdir(log_dir) if f.endswith(".wpilog") and "_sim" not in f])
    if not files:
        print("No .wpilog files found!")
        return

    # Use the most recent log by default, or all if --all flag
    if "--all" in sys.argv:
        for f in files:
            try:
                analyze_file(os.path.join(log_dir, f))
            except Exception as e:
                import traceback
                print(f"  ERROR analyzing {f}: {e}")
                traceback.print_exc()
    else:
        # Analyze last 3 logs and average
        use_files = files[-3:]
        print(f"Analyzing {len(use_files)} most recent logs (use --all for all)")
        all_drive = []
        all_turn = []
        for f in use_files:
            try:
                analyze_file(os.path.join(log_dir, f))
            except Exception as e:
                import traceback
                print(f"  ERROR analyzing {f}: {e}")
                traceback.print_exc()


if __name__ == "__main__":
    main()
