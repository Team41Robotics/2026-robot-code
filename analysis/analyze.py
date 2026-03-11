"""Analyze wpilog replay files for FRC robot diagnostics."""
import sys, struct, math, os
from collections import defaultdict
from wpiutil.log import DataLogReader

def read_doubles(reader, keys):
    """Extract time-series data for double-typed keys."""
    entries = {}
    data = defaultdict(list)
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in keys:
                entries[d.entry] = d.name
        elif record.isFinish():
            pass
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entries:
                name = entries[eid]
                try:
                    val = record.getDouble()
                    ts = record.getTimestamp() / 1e6  # microseconds -> seconds
                    data[name].append((ts, val))
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

def read_strings(reader, keys):
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
                    val = record.getString()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

def stats(series):
    """Compute min/max/mean/std for a time series."""
    if not series:
        return None
    vals = [v for _, v in series]
    n = len(vals)
    mn = min(vals)
    mx = max(vals)
    mean = sum(vals) / n
    std = (sum((v - mean)**2 for v in vals) / n) ** 0.5
    return {"n": n, "min": mn, "max": mx, "mean": mean, "std": std}

def pct_above(series, threshold):
    """Percentage of samples above threshold."""
    if not series:
        return 0
    vals = [v for _, v in series]
    return 100.0 * sum(1 for v in vals if v > threshold) / len(vals)

def analyze_file(path):
    print(f"\n{'='*80}")
    print(f"  ANALYZING: {os.path.basename(path)}")
    print(f"{'='*80}")

    # --- Pass 1: Timing + system ---
    timing_keys = [
        "RealOutputs/LoggedRobot/FullCycleMS",
        "RealOutputs/LoggedRobot/UserCodeMS",
        "/SystemStats/BatteryVoltage",
    ]
    reader = DataLogReader(path)
    timing = read_doubles(reader, set(timing_keys))

    cycle = timing.get("RealOutputs/LoggedRobot/FullCycleMS", [])
    user = timing.get("RealOutputs/LoggedRobot/UserCodeMS", [])
    batt = timing.get("/SystemStats/BatteryVoltage", [])

    print("\n--- LOOP TIMING ---")
    if cycle:
        s = stats(cycle)
        print(f"  Full cycle:  mean={s['mean']:.2f}ms  max={s['max']:.2f}ms  std={s['std']:.2f}ms  ({s['n']} samples)")
        overruns = pct_above(cycle, 20.0)
        print(f"  Loop overruns (>20ms): {overruns:.1f}%")
        bad_overruns = pct_above(cycle, 40.0)
        if bad_overruns > 0:
            print(f"  ** SEVERE overruns (>40ms): {bad_overruns:.1f}% **")
    if user:
        s = stats(user)
        print(f"  User code:   mean={s['mean']:.2f}ms  max={s['max']:.2f}ms  std={s['std']:.2f}ms")

    print("\n--- BATTERY ---")
    if batt:
        s = stats(batt)
        print(f"  Voltage: min={s['min']:.2f}V  max={s['max']:.2f}V  mean={s['mean']:.2f}V")
        brownout = pct_above([(t, -v) for t, v in batt], -7.0)  # % below 7V
        low = sum(1 for _, v in batt if v < 7.0)
        if s['min'] < 7.0:
            print(f"  ** BROWNOUT DETECTED: min voltage {s['min']:.2f}V **")
        elif s['min'] < 9.0:
            print(f"  ** LOW BATTERY WARNING: min voltage {s['min']:.2f}V **")

    # --- Pass 2: Swerve drive errors ---
    swerve_keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        swerve_keys.add(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec")
        swerve_keys.add(f"/RealOutputs/Swerve/{corner}/turnErrorRadians")
        swerve_keys.add(f"/Swerve/{corner}/DriveVelMetersPerSec")
        swerve_keys.add(f"/Swerve/{corner}/TurnVoltageVolts")
        swerve_keys.add(f"/Swerve/{corner}/DriveCurrentAmps")

    reader = DataLogReader(path)
    swerve = read_doubles(reader, swerve_keys)

    print("\n--- SWERVE DRIVE ERRORS ---")
    for corner in ["NE", "NW", "SE", "SW"]:
        de = swerve.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        te = swerve.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])
        if de:
            ds = stats(de)
            print(f"  {corner} drive error: mean={ds['mean']:.4f} m/s  max={ds['max']:.4f}  std={ds['std']:.4f}")
        if te:
            ts_s = stats(te)
            print(f"  {corner} turn  error: mean={ts_s['mean']:.4f} rad   max={ts_s['max']:.4f}  std={ts_s['std']:.4f}")

    # Check for any module with consistently high errors
    print("\n  Module health check:")
    for corner in ["NE", "NW", "SE", "SW"]:
        de = swerve.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        te = swerve.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])
        issues = []
        if de:
            ds = stats(de)
            if abs(ds['mean']) > 0.5:
                issues.append(f"high avg drive error ({ds['mean']:.3f} m/s)")
            if ds['max'] > 2.0:
                issues.append(f"drive error spike ({ds['max']:.3f} m/s)")
        if te:
            ts_s = stats(te)
            if abs(ts_s['mean']) > 0.1:
                issues.append(f"high avg turn error ({ts_s['mean']:.3f} rad)")
            if ts_s['max'] > 0.5:
                issues.append(f"turn error spike ({ts_s['max']:.3f} rad = {math.degrees(ts_s['max']):.1f} deg)")
        if issues:
            print(f"    ** {corner}: {', '.join(issues)} **")
        else:
            print(f"    {corner}: OK")

    # --- Pass 3: Shooter ---
    shooter_keys = {
        "/RealOutputs//Shooter/targetFlywheelRPM",
        "/RealOutputs//Shooter/targetTurretPosRadians",
        "/RealOutputs//Shooter/turretProfilePosRadians",
        "/RealOutputs//Shooter/targetHoodPosRadians",
        "/RealOutputs//Shooter/hoodProfilePosRadians",
        "/RealOutputs//Shooter/flywheelErrorRPM",
        "/RealOutputs//Shooter/turretErrorRadians",
        "/RealOutputs//Shooter/hoodErrorRadians",
        "//Shooter/FlywheelVelocityRPM",
        "//Shooter/TurretPosRadians",
        "//Shooter/HoodPosRadians",
        "//Shooter/FlywheelCurrentAmps",
        "//Shooter/TurretCurrentAmps",
        "//Shooter/HoodCurrentAmps",
    }
    reader = DataLogReader(path)
    shooter = read_doubles(reader, shooter_keys)

    print("\n--- SHOOTER ---")
    fw_err = shooter.get("/RealOutputs//Shooter/flywheelErrorRPM", [])
    turret_err = shooter.get("/RealOutputs//Shooter/turretErrorRadians", [])
    hood_err = shooter.get("/RealOutputs//Shooter/hoodErrorRadians", [])
    fw_target = shooter.get("/RealOutputs//Shooter/targetFlywheelRPM", [])
    fw_actual = shooter.get("//Shooter/FlywheelVelocityRPM", [])

    if fw_err:
        s = stats(fw_err)
        print(f"  Flywheel error: mean={s['mean']:.1f} RPM  max={s['max']:.1f}  std={s['std']:.1f}")
    if turret_err:
        s = stats(turret_err)
        print(f"  Turret error:   mean={s['mean']:.4f} rad ({math.degrees(s['mean']):.2f} deg)  max={s['max']:.4f} rad ({math.degrees(s['max']):.2f} deg)")
    if hood_err:
        s = stats(hood_err)
        print(f"  Hood error:     mean={s['mean']:.4f} rad ({math.degrees(s['mean']):.2f} deg)  max={s['max']:.4f} rad ({math.degrees(s['max']):.2f} deg)")

    # Flywheel tracking: when target > 500 RPM, how well does actual track?
    if fw_target and fw_actual:
        # Align by finding closest timestamps
        target_active = [(t, v) for t, v in fw_target if v > 500]
        if target_active:
            print(f"\n  Flywheel active time: {len(target_active)} samples with target > 500 RPM")
            # Find actual values closest to active target timestamps
            actual_dict = {}
            for t, v in fw_actual:
                actual_dict[round(t, 3)] = v
            tracking_errors = []
            for t, tgt in target_active:
                act = actual_dict.get(round(t, 3))
                if act is not None:
                    tracking_errors.append(abs(tgt - act))
            if tracking_errors:
                mean_err = sum(tracking_errors) / len(tracking_errors)
                max_err = max(tracking_errors)
                pct_within_100 = 100.0 * sum(1 for e in tracking_errors if e < 100) / len(tracking_errors)
                print(f"  Flywheel tracking: mean error={mean_err:.1f} RPM  max={max_err:.1f} RPM")
                print(f"  Within 100 RPM of target: {pct_within_100:.1f}%")
                if pct_within_100 < 80:
                    print(f"  ** FLYWHEEL TRACKING POOR - needs PID tuning **")

    # Flywheel current draw
    fw_curr = shooter.get("//Shooter/FlywheelCurrentAmps", [])
    if fw_curr:
        s = stats(fw_curr)
        print(f"  Flywheel current: mean={s['mean']:.1f}A  max={s['max']:.1f}A")
        if s['max'] > 80:
            print(f"  ** HIGH FLYWHEEL CURRENT ({s['max']:.1f}A) - check for stall **")

    turret_curr = shooter.get("//Shooter/TurretCurrentAmps", [])
    if turret_curr:
        s = stats(turret_curr)
        print(f"  Turret current:   mean={s['mean']:.1f}A  max={s['max']:.1f}A")

    hood_curr = shooter.get("//Shooter/HoodCurrentAmps", [])
    if hood_curr:
        s = stats(hood_curr)
        print(f"  Hood current:     mean={s['mean']:.1f}A  max={s['max']:.1f}A")

    # --- Pass 4: Vision ---
    vision_str_keys = {
        "/RealOutputs//Vision/DuckyNE/method",
        "/RealOutputs//Vision/DuckySE/method",
    }
    reader = DataLogReader(path)
    vision_str = read_strings(reader, vision_str_keys)

    print("\n--- VISION ---")
    for cam in ["DuckyNE", "DuckySE"]:
        methods = vision_str.get(f"/RealOutputs//Vision/{cam}/method", [])
        if methods:
            counts = defaultdict(int)
            for _, m in methods:
                counts[m] += 1
            total = len(methods)
            print(f"  {cam}: {total} vision updates")
            for m, c in sorted(counts.items(), key=lambda x: -x[1]):
                print(f"    {m}: {c} ({100*c/total:.1f}%)")

    # --- Pass 5: Intake ---
    intake_keys = {
        "//Intake/JointPosRadians",
        "//Intake/JointVelRadiansPerSec",
        "//Intake/JointCurrentAmps",
        "//Intake/IntakeCurrentAmps",
    }
    # Check if intake keys exist
    reader = DataLogReader(path)
    all_keys = set()
    for record in reader:
        if record.isStart():
            all_keys.add(record.getStartData().name)

    # Find intake-related keys
    intake_found = [k for k in all_keys if "Intake" in k or "intake" in k]
    if intake_found:
        reader = DataLogReader(path)
        intake = read_doubles(reader, set(intake_found))
        print("\n--- INTAKE ---")
        for key in sorted(intake.keys()):
            s = stats(intake[key])
            if s:
                short = key.split("/")[-1]
                print(f"  {short}: mean={s['mean']:.3f}  min={s['min']:.3f}  max={s['max']:.3f}")

    # --- Pass 6: Power distribution ---
    pwr_keys = {
        "/PowerDistribution/TotalCurrent",
        "/PowerDistribution/Voltage",
    }
    reader = DataLogReader(path)
    pwr = read_doubles(reader, pwr_keys)

    print("\n--- POWER ---")
    total_curr = pwr.get("/PowerDistribution/TotalCurrent", [])
    if total_curr:
        s = stats(total_curr)
        print(f"  Total current: mean={s['mean']:.1f}A  max={s['max']:.1f}A")
        if s['max'] > 120:
            print(f"  ** HIGH TOTAL CURRENT ({s['max']:.1f}A) - risk of breaker trip **")
        high_draw = pct_above(total_curr, 80)
        if high_draw > 5:
            print(f"  ** {high_draw:.1f}% of match above 80A **")

    pdp_v = pwr.get("/PowerDistribution/Voltage", [])
    if pdp_v:
        s = stats(pdp_v)
        print(f"  PDH voltage:   min={s['min']:.2f}V  max={s['max']:.2f}V  mean={s['mean']:.2f}V")

    # --- Pass 7: Odometry sanity ---
    odom_keys = {
        "/RealOutputs//Odom/xMeters",
        "/RealOutputs//Odom/yMeters",
        "/RealOutputs//Odom/rotRadians",
    }
    reader = DataLogReader(path)
    odom = read_doubles(reader, odom_keys)

    print("\n--- ODOMETRY ---")
    ox = odom.get("/RealOutputs//Odom/xMeters", [])
    oy = odom.get("/RealOutputs//Odom/yMeters", [])
    if ox and oy:
        sx = stats(ox)
        sy = stats(oy)
        print(f"  X range: {sx['min']:.2f} to {sx['max']:.2f} m")
        print(f"  Y range: {sy['min']:.2f} to {sy['max']:.2f} m")
        # Check for wild jumps (vision correction artifacts)
        jumps_x = 0
        jumps_y = 0
        for i in range(1, len(ox)):
            if abs(ox[i][1] - ox[i-1][1]) > 1.0:
                jumps_x += 1
        for i in range(1, len(oy)):
            if abs(oy[i][1] - oy[i-1][1]) > 1.0:
                jumps_y += 1
        if jumps_x > 0 or jumps_y > 0:
            print(f"  ** POSE JUMPS detected: {jumps_x} in X, {jumps_y} in Y (>1m between frames) **")
            print(f"     This may indicate vision correction too aggressive or bad vision estimates")

    orot = odom.get("/RealOutputs//Odom/rotRadians", [])
    if orot:
        sr = stats(orot)
        print(f"  Heading range: {math.degrees(sr['min']):.1f} to {math.degrees(sr['max']):.1f} deg")

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

    print(f"Found {len(files)} log files in {log_dir}")
    for f in files:
        try:
            analyze_file(os.path.join(log_dir, f))
        except Exception as e:
            print(f"  ERROR analyzing {f}: {e}")


if __name__ == "__main__":
    main()
