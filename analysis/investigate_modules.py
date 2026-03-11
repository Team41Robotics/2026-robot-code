"""Investigate why NE/NW/SE have ~2m/s drive error while SW tracks well."""
import sys, math, os
from collections import defaultdict
from wpiutil.log import DataLogReader
import numpy as np

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
    if not series:
        return np.array([]), np.array([])
    return np.array([x[0] for x in series]), np.array([x[1] for x in series])

def align_by_time(series_a, series_b, tol=0.015):
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


def analyze_file(path):
    print(f"\n{'='*90}")
    print(f"  MODULE TRACKING INVESTIGATION: {os.path.basename(path)}")
    print(f"{'='*90}")

    reader = DataLogReader(path)
    enabled_data = read_booleans(reader, {"/DriverStation/Enabled"})
    enabled = enabled_data.get("/DriverStation/Enabled", [])
    enabled_periods = find_enabled_periods(enabled)

    keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        keys.add(f"/Swerve/{corner}/DriveVoltageVolts")
        keys.add(f"/Swerve/{corner}/DriveVelMetersPerSec")
        keys.add(f"/Swerve/{corner}/DrivePosMeters")
        keys.add(f"/Swerve/{corner}/DriveCurrentAmps")
        keys.add(f"/Swerve/{corner}/DriveBusVoltageVolts")
        keys.add(f"/Swerve/{corner}/DriveBusCurrentAmps")
        keys.add(f"/Swerve/{corner}/TurnAbsPosRadians")
        keys.add(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{corner}/driveFFVolts")
        keys.add(f"/RealOutputs/Swerve/{corner}/setpointVelMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{corner}/targetVelMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{corner}/turnErrorRadians")
        keys.add(f"/RealOutputs/Swerve/{corner}/targetAngRadians")
        keys.add(f"/RealOutputs/Swerve/{corner}/setpointAngRadians")

    reader = DataLogReader(path)
    data = read_doubles(reader, keys)

    for corner in ["NE", "NW", "SE", "SW"]:
        print(f"\n{'-'*60}")
        print(f"  [{corner}] DETAILED DIAGNOSTICS")
        print(f"{'-'*60}")

        err_raw = data.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        vel = data.get(f"/Swerve/{corner}/DriveVelMetersPerSec", [])
        volt = data.get(f"/Swerve/{corner}/DriveVoltageVolts", [])
        ff = data.get(f"/RealOutputs/Swerve/{corner}/driveFFVolts", [])
        target = data.get(f"/RealOutputs/Swerve/{corner}/targetVelMetersPerSec", [])
        setpoint = data.get(f"/RealOutputs/Swerve/{corner}/setpointVelMetersPerSec", [])
        curr = data.get(f"/Swerve/{corner}/DriveCurrentAmps", [])
        bus_v = data.get(f"/Swerve/{corner}/DriveBusVoltageVolts", [])
        pos = data.get(f"/Swerve/{corner}/DrivePosMeters", [])
        turn_err = data.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])

        err_en = filter_enabled(err_raw, enabled_periods)
        vel_en = filter_enabled(vel, enabled_periods)
        target_en = filter_enabled(target, enabled_periods)
        setpoint_en = filter_enabled(setpoint, enabled_periods)
        volt_en = filter_enabled(volt, enabled_periods)
        ff_en = filter_enabled(ff, enabled_periods)
        curr_en = filter_enabled(curr, enabled_periods)
        bus_v_en = filter_enabled(bus_v, enabled_periods)
        turn_err_en = filter_enabled(turn_err, enabled_periods)

        if not err_en:
            print("  No data")
            continue

        _, err_vals = to_arrays(err_en)

        # 1. Error DIRECTION analysis (not just absolute)
        # error = measured - target: positive = going too fast, negative = too slow
        pos_err = np.sum(err_vals > 0.1)
        neg_err = np.sum(err_vals < -0.1)
        near_zero = np.sum(np.abs(err_vals) <= 0.1)
        print(f"\n  Error direction (error = measured - target):")
        print(f"    Too fast (>0.1):  {pos_err} ({100*pos_err/len(err_vals):.1f}%)")
        print(f"    Too slow (<-0.1): {neg_err} ({100*neg_err/len(err_vals):.1f}%)")
        print(f"    On target:        {near_zero} ({100*near_zero/len(err_vals):.1f}%)")
        print(f"    Mean signed error: {np.mean(err_vals):.3f} m/s")
        print(f"    Mean |error|:      {np.mean(np.abs(err_vals)):.3f} m/s")

        # 2. Compare target vs setpoint vs measured
        if target_en and setpoint_en and vel_en:
            t1, tgt, sp = align_by_time(target_en, setpoint_en)
            t2, tgt2, meas = align_by_time(target_en, vel_en)
            if len(t1) > 50 and len(t2) > 50:
                tgt_sp_diff = np.abs(tgt - sp)
                print(f"\n  Target vs Setpoint vs Measured:")
                print(f"    |target - setpoint|: mean={np.mean(tgt_sp_diff):.4f}  max={np.max(tgt_sp_diff):.4f}")
                print(f"    target range: [{np.min(tgt):.2f}, {np.max(tgt):.2f}] m/s")
                print(f"    measured range: [{np.min(meas):.2f}, {np.max(meas):.2f}] m/s")

                # Is setpoint tracking target well? (profile not limiting?)
                if np.mean(tgt_sp_diff) > 0.5:
                    print(f"    ** Setpoint lags target — drive profile constraints may be limiting **")

        # 3. Voltage analysis: is the motor getting enough voltage?
        if volt_en and ff_en:
            t_v, total_v, ff_v = align_by_time(volt_en, ff_en)
            if len(t_v) > 50:
                pid_v = total_v - ff_v
                print(f"\n  Voltage breakdown:")
                print(f"    Total voltage: mean={np.mean(np.abs(total_v)):.2f}V  max={np.max(np.abs(total_v)):.2f}V")
                print(f"    FF voltage:    mean={np.mean(np.abs(ff_v)):.2f}V  max={np.max(np.abs(ff_v)):.2f}V")
                print(f"    PID voltage:   mean={np.mean(pid_v):.2f}V  mean|PID|={np.mean(np.abs(pid_v)):.2f}V")

                # Is voltage saturating?
                saturated = np.sum(np.abs(total_v) > 11.0)
                print(f"    Saturated (>11V): {saturated} ({100*saturated/len(total_v):.1f}%)")

        # 4. Current analysis
        if curr_en:
            _, curr_vals = to_arrays(curr_en)
            print(f"\n  Current:")
            print(f"    Mean: {np.mean(np.abs(curr_vals)):.1f}A  Max: {np.max(np.abs(curr_vals)):.1f}A")
            # Current vs voltage correlation (motor health)
            if volt_en:
                t_cv, curr_a, volt_a = align_by_time(curr_en, volt_en)
                if len(t_cv) > 50:
                    moving = np.abs(volt_a) > 1.0
                    if np.sum(moving) > 20:
                        # V/I ratio when moving gives effective resistance
                        r_eff = np.mean(np.abs(volt_a[moving]) / (np.abs(curr_a[moving]) + 0.1))
                        print(f"    Effective V/I (when driving): {r_eff:.3f}")

        # 5. Bus voltage during high-demand
        if bus_v_en:
            _, bv_vals = to_arrays(bus_v_en)
            print(f"\n  Bus voltage: min={np.min(bv_vals):.2f}V  mean={np.mean(bv_vals):.2f}V")

        # 6. Turn error when drive error is high (misaligned wheel = lost drive force)
        if turn_err_en and err_en:
            t_te, terr, derr = align_by_time(turn_err_en, err_en)
            if len(t_te) > 50:
                high_derr = np.abs(derr) > 1.0
                if np.sum(high_derr) > 10:
                    terr_when_high = np.abs(terr[high_derr])
                    terr_when_low = np.abs(terr[~high_derr])
                    print(f"\n  Turn error correlation:")
                    print(f"    Turn error when |drive err| > 1m/s: {np.mean(terr_when_high):.3f} rad ({math.degrees(np.mean(terr_when_high)):.1f}deg)")
                    print(f"    Turn error when |drive err| < 1m/s: {np.mean(terr_when_low):.3f} rad ({math.degrees(np.mean(terr_when_low)):.1f}deg)")
                    if np.mean(terr_when_high) > np.mean(terr_when_low) * 2:
                        print(f"    ** Drive error correlated with turn misalignment **")

        # 7. Check if position is actually moving (encoder health)
        if pos:
            pos_en = filter_enabled(pos, enabled_periods)
            if pos_en:
                _, pos_vals = to_arrays(pos_en)
                total_dist = np.sum(np.abs(np.diff(pos_vals)))
                print(f"\n  Encoder health:")
                print(f"    Total distance traveled: {total_dist:.1f}m")
                # Check for position stalls (no movement for >0.5s while voltage applied)
                if volt_en and len(pos_en) > 50:
                    t_p, pos_a, volt_a2 = align_by_time(pos_en, volt_en)
                    if len(t_p) > 50:
                        stall_count = 0
                        window = 25  # ~0.5s at 50Hz
                        for i in range(window, len(pos_a)):
                            pos_range = np.max(pos_a[i-window:i]) - np.min(pos_a[i-window:i])
                            volt_mean = np.mean(np.abs(volt_a2[i-window:i]))
                            if pos_range < 0.01 and volt_mean > 2.0:
                                stall_count += 1
                        if stall_count > 0:
                            print(f"    ** {stall_count} stall-like samples (no movement with >2V applied) **")

        # 8. Time-windowed error analysis (is it getting worse over time?)
        if len(err_en) > 200:
            _, err_v = to_arrays(err_en)
            t_arr, _ = to_arrays(err_en)
            n = len(err_v)
            q1 = np.mean(np.abs(err_v[:n//4]))
            q2 = np.mean(np.abs(err_v[n//4:n//2]))
            q3 = np.mean(np.abs(err_v[n//2:3*n//4]))
            q4 = np.mean(np.abs(err_v[3*n//4:]))
            print(f"\n  Error over time (quartiles):")
            print(f"    Q1: {q1:.3f}  Q2: {q2:.3f}  Q3: {q3:.3f}  Q4: {q4:.3f}")
            if q4 > q1 * 2:
                print(f"    ** Error increasing over match — thermal/battery degradation? **")

    print()


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../logs"
    if os.path.isfile(log_dir):
        analyze_file(log_dir)
        return
    files = sorted([f for f in os.listdir(log_dir) if f.endswith(".wpilog") and "_sim" not in f])
    if files:
        # Most recent
        analyze_file(os.path.join(log_dir, files[-1]))

if __name__ == "__main__":
    main()
