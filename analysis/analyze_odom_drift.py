"""Analyze odometry drift: compare odom pose to vision estimates, check tracking errors."""
import sys, os, math
from collections import defaultdict
from wpiutil.log import DataLogReader

# ── helpers ──────────────────────────────────────────────────────────────────

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
                    data[entries[eid]].append((record.getTimestamp() / 1e6, record.getDouble()))
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
                    data[entries[eid]].append((record.getTimestamp() / 1e6, record.getString()))
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
                    data[entries[eid]].append((record.getTimestamp() / 1e6, record.getBoolean()))
                except Exception:
                    pass
    return data


def read_struct_pose2d(reader, keys):
    """Read Pose2d structs (x, y in first 16 bytes as two doubles, then rotation as one double)."""
    entries = {}
    data = defaultdict(list)
    import struct as st
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in keys:
                entries[d.entry] = d.name
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entries:
                try:
                    raw = record.getRaw()
                    if len(raw) >= 24:
                        x, y, cos, sin = st.unpack_from('<dddd', raw, 0)
                        angle = math.atan2(sin, cos)
                        data[entries[eid]].append((record.getTimestamp() / 1e6, x, y, angle))
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
    if start is not None and enabled_series:
        periods.append((start, enabled_series[-1][0]))
    return periods


def interp(series, t):
    """Linear interpolate a [(t, v)] series at time t."""
    if not series:
        return None
    if t <= series[0][0]:
        return series[0][1]
    if t >= series[-1][0]:
        return series[-1][1]
    for i in range(1, len(series)):
        if series[i][0] >= t:
            t0, v0 = series[i-1]
            t1, v1 = series[i]
            frac = (t - t0) / (t1 - t0)
            return v0 + frac * (v1 - v0)
    return series[-1][1]


def percentile(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    idx = p / 100.0 * (len(s) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (idx - lo) * (s[hi] - s[lo])


def stats(vals, label, unit=""):
    if not vals:
        print(f"    {label}: no data")
        return
    print(f"    {label}: mean={sum(vals)/len(vals):.3f}{unit}  "
          f"p50={percentile(vals,50):.3f}{unit}  "
          f"p90={percentile(vals,90):.3f}{unit}  "
          f"p99={percentile(vals,99):.3f}{unit}  "
          f"max={max(vals):.3f}{unit}  n={len(vals)}")


# ── main analysis ─────────────────────────────────────────────────────────────

def analyze_file(path):
    print(f"\n{'='*90}")
    print(f"  ODOM DRIFT: {os.path.basename(path)}")
    print(f"{'='*90}")

    # Enabled periods
    reader = DataLogReader(path)
    en_data = read_booleans(reader, {"/DriverStation/Enabled"})
    enabled = en_data.get("/DriverStation/Enabled", [])
    enabled_periods = find_enabled_periods(enabled)
    if not enabled_periods:
        print("  No enabled periods found — skipping")
        return
    total_en = sum(e - s for s, e in enabled_periods)
    print(f"  Enabled: {total_en:.1f}s across {len(enabled_periods)} period(s)")

    def during_match(t):
        return any(s <= t <= e for s, e in enabled_periods)

    # ── 1. ODOM vs VISION DRIFT ──────────────────────────────────────────────
    print("\n--- 1. ODOM vs VISION POSE DRIFT ---")
    pose_keys = {
        "/RealOutputs//Odom/pose",
        "/RealOutputs//Vision/DuckyNE/estimatedPose",
        "/RealOutputs//Vision/DuckySE/estimatedPose",
    }
    reader = DataLogReader(path)
    poses = read_struct_pose2d(reader, pose_keys)

    odom_poses = poses.get("/RealOutputs//Odom/pose", [])
    ne_poses = poses.get("/RealOutputs//Vision/DuckyNE/estimatedPose", [])
    se_poses = poses.get("/RealOutputs//Vision/DuckySE/estimatedPose", [])

    # Build odom XY series for interpolation
    odom_x = [(t, x) for t, x, y, a in odom_poses]
    odom_y = [(t, y) for t, x, y, a in odom_poses]

    for cam_name, cam_poses in [("DuckyNE", ne_poses), ("DuckySE", se_poses)]:
        errors = []
        for t, vx, vy, va in cam_poses:
            if not during_match(t):
                continue
            ox = interp(odom_x, t)
            oy = interp(odom_y, t)
            if ox is None or oy is None:
                continue
            err = math.hypot(vx - ox, vy - oy)
            errors.append(err)

        if errors:
            print(f"  {cam_name} vs odom XY error during match:")
            stats(errors, "drift", "m")
            big = [(t, math.hypot(vx - interp(odom_x, t), vy - interp(odom_y, t)))
                   for t, vx, vy, va in cam_poses
                   if during_match(t) and interp(odom_x, t) is not None]
            large = [(t, e) for t, e in big if e > 0.5]
            if large:
                print(f"    {len(large)} vision corrections >0.5m:")
                for t, e in large[:10]:
                    print(f"      t={t:.2f}s  error={e:.3f}m")
        else:
            print(f"  {cam_name}: no vision poses during match")

    # ── 2. ODOM HEADING vs IMU YAW ──────────────────────────────────────────
    print("\n--- 2. ODOM HEADING vs IMU YAW ---")
    reader = DataLogReader(path)
    d_keys = {
        "/RealOutputs//Odom/rotRadians",
        "/RealOutputs//Odom/imuYawRadians",
        "//IMU/YawRadians",
    }
    dbl = read_doubles(reader, d_keys)
    odom_rot = dbl.get("/RealOutputs//Odom/rotRadians", [])
    odom_imu = dbl.get("/RealOutputs//Odom/imuYawRadians", [])

    if odom_rot and odom_imu:
        # Compare poseEst heading to raw IMU heading
        imu_interp = [(t, v) for t, v in odom_imu]
        heading_diffs = []
        for t, rot in odom_rot:
            if not during_match(t):
                continue
            imu_val = interp(imu_interp, t)
            if imu_val is None:
                continue
            diff = abs(math.atan2(math.sin(rot - imu_val), math.cos(rot - imu_val)))
            heading_diffs.append(diff)
        if heading_diffs:
            stats(heading_diffs, "odom_rot vs imu_yaw diff", "rad")
            if max(heading_diffs) > 0.1:
                print(f"    ** Vision heading correction active (max={math.degrees(max(heading_diffs)):.1f}°) **")
        else:
            print("  No overlapping heading data during match")
    else:
        print("  Missing heading data")

    # ── 3. TURN TRACKING ERROR ──────────────────────────────────────────────
    print("\n--- 3. TURN TRACKING ERROR (abs encoder vs target) ---")
    reader = DataLogReader(path)
    turn_keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        turn_keys.add(f"/RealOutputs/Swerve/{corner}/turnErrorRadians")
        turn_keys.add(f"/RealOutputs/Swerve/{corner}/targetAngRadians")
    turn_dbl = read_doubles(reader, turn_keys)

    for corner in ["NE", "NW", "SE", "SW"]:
        errs = [(t, abs(v)) for t, v in turn_dbl.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])
                if during_match(t)]
        if errs:
            vals = [v for _, v in errs]
            big = [(t, v) for t, v in errs if v > 0.2]
            p90 = percentile(vals, 90)
            mean = sum(vals) / len(vals)
            print(f"  {corner}: mean={math.degrees(mean):.1f}°  p90={math.degrees(p90):.1f}°  "
                  f"max={math.degrees(max(vals)):.1f}°  >0.2rad={len(big)}")
        else:
            print(f"  {corner}: no data")

    # ── 4. DRIVE VELOCITY TRACKING ERROR ───────────────────────────────────
    print("\n--- 4. DRIVE VELOCITY TRACKING ERROR ---")
    reader = DataLogReader(path)
    drive_err_keys = {f"/RealOutputs/Swerve/{c}/driveErrorMetersPerSec" for c in ["NE", "NW", "SE", "SW"]}
    drive_vel_keys = {f"/RealOutputs/Swerve/{c}/targetVelMetersPerSec" for c in ["NE", "NW", "SE", "SW"]}
    drive_dbl = read_doubles(reader, drive_err_keys | drive_vel_keys)

    for corner in ["NE", "NW", "SE", "SW"]:
        errs = [(t, abs(v)) for t, v in drive_dbl.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
                if during_match(t)]
        vels = [(t, abs(v)) for t, v in drive_dbl.get(f"/RealOutputs/Swerve/{corner}/targetVelMetersPerSec", [])
                if during_match(t)]
        if errs and vels:
            vals = [v for _, v in errs]
            moving = [(t, e) for t, e in errs if interp([(tv, vv) for tv, vv in vels], t) > 0.3]
            if moving:
                mv = [v for _, v in moving]
                p90 = percentile(mv, 90)
                mean = sum(mv) / len(mv)
                print(f"  {corner} (while moving): mean={mean:.3f}m/s  p90={p90:.3f}m/s  max={max(mv):.3f}m/s")
            else:
                print(f"  {corner}: no significant motion data")
        else:
            print(f"  {corner}: no data")

    # ── 5. VISION CORRECTION RATE ───────────────────────────────────────────
    print("\n--- 5. VISION UPDATE RATE & METHOD ---")
    reader = DataLogReader(path)
    method_keys = {
        "/RealOutputs//Vision/DuckyNE/method",
        "/RealOutputs//Vision/DuckySE/method",
    }
    methods = read_strings(reader, method_keys)

    for cam in ["DuckyNE", "DuckySE"]:
        series = [(t, m) for t, m in methods.get(f"/RealOutputs//Vision/{cam}/method", [])
                  if during_match(t)]
        if not series:
            print(f"  {cam}: no data during match")
            continue
        counts = defaultdict(int)
        for _, m in series:
            counts[m] += 1
        total = len(series)
        match_dur = total_en
        useful = total - counts["none"]
        rate = useful / match_dur if match_dur > 0 else 0
        print(f"  {cam}: total={total}  useful={useful} ({100*useful/total:.0f}%)  "
              f"rate={rate:.1f}Hz  methods={dict(counts)}")

    # ── 6. ODOM JUMP DETECTION ─────────────────────────────────────────────
    print("\n--- 6. ODOM DISCONTINUITIES (pose jumps >0.1m/loop) ---")
    jumps = []
    for i in range(1, len(odom_poses)):
        t0, x0, y0, _ = odom_poses[i-1]
        t1, x1, y1, _ = odom_poses[i]
        if not during_match(t1):
            continue
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            continue
        dist = math.hypot(x1 - x0, y1 - y0)
        vel_implied = dist / dt
        if vel_implied > 8.0:  # > 8 m/s is physically impossible
            jumps.append((t1, dist, vel_implied))

    if jumps:
        print(f"  {len(jumps)} impossible jumps (>8 m/s implied):")
        for t, d, v in jumps[:10]:
            print(f"    t={t:.2f}s  Δ={d:.3f}m  implied={v:.1f}m/s")
    else:
        print("  No impossible jumps found")

    # Also check heading jumps
    heading_jumps = []
    for i in range(1, len(odom_poses)):
        t0, _, _, a0 = odom_poses[i-1]
        t1, _, _, a1 = odom_poses[i]
        if not during_match(t1):
            continue
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            continue
        da = abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
        omega = da / dt
        if omega > 30.0:  # >30 rad/s impossible
            heading_jumps.append((t1, math.degrees(da), omega))

    if heading_jumps:
        print(f"  {len(heading_jumps)} impossible heading jumps (>30 rad/s):")
        for t, da, w in heading_jumps[:10]:
            print(f"    t={t:.2f}s  Δ={da:.1f}°  omega={w:.1f}rad/s")
    else:
        print("  No impossible heading jumps found")

    # ── 7. BROWNOUT CORRELATION ─────────────────────────────────────────────
    print("\n--- 7. ODOM DRIFT RATE VS VOLTAGE ---")
    reader = DataLogReader(path)
    batt_keys = {"/SystemStats/BatteryVoltage"}
    batt_dbl = read_doubles(reader, batt_keys)
    batt = batt_dbl.get("/SystemStats/BatteryVoltage", [])

    if batt and ne_poses and odom_x:
        # Bucket odom drift by battery voltage
        buckets = defaultdict(list)
        for t, vx, vy, _ in ne_poses:
            if not during_match(t):
                continue
            bv = interp(batt, t)
            if bv is None:
                continue
            ox = interp(odom_x, t)
            oy = interp(odom_y, t)
            if ox is None:
                continue
            err = math.hypot(vx - ox, vy - oy)
            bucket = int(bv)
            buckets[bucket].append(err)

        for v in sorted(buckets.keys(), reverse=True):
            errs = buckets[v]
            if errs:
                print(f"  Battery {v}-{v+1}V: n={len(errs):3d}  "
                      f"mean_drift={sum(errs)/len(errs):.3f}m  "
                      f"p90={percentile(errs,90):.3f}m")
    else:
        print("  Insufficient data for voltage correlation")


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../logs"

    if os.path.isfile(log_dir):
        analyze_file(log_dir)
        return

    files = sorted([f for f in os.listdir(log_dir)
                    if f.endswith(".wpilog") and "_sim" not in f])
    if not files:
        print("No .wpilog files found!")
        return

    print(f"Found {len(files)} log files in {log_dir}")
    for f in files:
        try:
            analyze_file(os.path.join(log_dir, f))
        except Exception as e:
            import traceback
            print(f"  ERROR: {f}: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()
