"""Analyze wheel slip by comparing stator current force to estimated traction limit,
and correlating high-current events with odometry drift."""
import sys, os, math
from collections import defaultdict
from wpiutil.log import DataLogReader
import numpy as np

# -- Robot constants ----------------------------------------------------------
KRAKEN_KT = 7.09 / 366.0  # Nm/A (stall torque / stall current)
DRIVE_RATIO = 5.27  # motor:wheel
WHEEL_RAD = 2 * 2.54 / 100.0  # 2 inches in meters
STATOR_LIMIT = 90.0  # A

# Traction estimates
ROBOT_MASS_KG = 59.0  # ~130 lbs with battery/bumpers
MU_CARPET = 1.1  # rubber on carpet
NORMAL_FORCE_PER_WHEEL = ROBOT_MASS_KG * 9.81 / 4.0
MAX_TRACTION_FORCE = MU_CARPET * NORMAL_FORCE_PER_WHEEL

def current_to_wheel_force(stator_amps):
    """Convert stator current to force at wheel contact patch."""
    motor_torque = abs(stator_amps) * KRAKEN_KT
    wheel_torque = motor_torque * DRIVE_RATIO
    return wheel_torque / WHEEL_RAD

# -- Log reading helpers ------------------------------------------------------

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

def find_enabled_periods(s):
    periods, start = [], None
    for t, v in s:
        if v and start is None: start = t
        elif not v and start is not None: periods.append((start, t)); start = None
    if start is not None: periods.append((start, s[-1][0]))
    return periods

def filter_enabled(series, ep):
    out = []
    for s, e in ep:
        out.extend([(t, v) for t, v in series if s <= t <= e])
    return out

def to_arrays(series):
    if not series: return np.array([]), np.array([])
    return np.array([x[0] for x in series]), np.array([x[1] for x in series])

def align_by_time(sa, sb, tol=0.015):
    if not sa or not sb: return np.array([]), np.array([]), np.array([])
    ta, va = to_arrays(sa)
    tb, vb = to_arrays(sb)
    times, ao, bo = [], [], []
    j = 0
    for i in range(len(ta)):
        while j < len(tb)-1 and tb[j+1] <= ta[i]: j += 1
        if j < len(tb) and abs(tb[j]-ta[i]) < tol:
            times.append(ta[i]); ao.append(va[i]); bo.append(vb[j])
    return np.array(times), np.array(ao), np.array(bo)

# -- Analysis -----------------------------------------------------------------

def analyze(path):
    print(f"\n{'='*90}")
    print(f"  WHEEL SLIP ANALYSIS: {os.path.basename(path)}")
    print(f"{'='*90}")

    # Theoretical limits
    force_at_limit = current_to_wheel_force(STATOR_LIMIT)
    slip_threshold_amps = MAX_TRACTION_FORCE * WHEEL_RAD / (KRAKEN_KT * DRIVE_RATIO)
    print(f"\n  Theoretical limits:")
    print(f"    Force at {STATOR_LIMIT:.0f}A stator: {force_at_limit:.1f} N")
    print(f"    Max traction (static, u={MU_CARPET}): {MAX_TRACTION_FORCE:.1f} N")
    print(f"    Slip threshold current: {slip_threshold_amps:.1f} A")
    print(f"    {'** CURRENT LIMIT EXCEEDS TRACTION **' if force_at_limit > MAX_TRACTION_FORCE else 'Current limit within traction'}")
    print(f"    Margin: {(force_at_limit / MAX_TRACTION_FORCE - 1) * 100:+.0f}%")

    reader = DataLogReader(path)
    en = read_booleans(reader, {"/DriverStation/Enabled"}).get("/DriverStation/Enabled", [])
    ep = find_enabled_periods(en)
    if not ep:
        print("  No enabled periods")
        return

    keys = set()
    for c in ["NE", "NW", "SE", "SW"]:
        keys.add(f"/Swerve/{c}/DriveCurrentAmps")
        keys.add(f"/Swerve/{c}/DriveVelMetersPerSec")
        keys.add(f"/Swerve/{c}/DrivePosMeters")
        keys.add(f"/RealOutputs/Swerve/{c}/setpointVelMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{c}/driveErrorMetersPerSec")

    reader = DataLogReader(path)
    d = read_doubles(reader, keys)

    print(f"\n  Per-module slip analysis:")
    print(f"  {'-'*80}")

    for c in ["NE", "NW", "SE", "SW"]:
        curr = filter_enabled(d.get(f"/Swerve/{c}/DriveCurrentAmps", []), ep)
        vel = filter_enabled(d.get(f"/Swerve/{c}/DriveVelMetersPerSec", []), ep)
        pos = filter_enabled(d.get(f"/Swerve/{c}/DrivePosMeters", []), ep)
        setpoint = filter_enabled(d.get(f"/RealOutputs/Swerve/{c}/setpointVelMetersPerSec", []), ep)
        err = filter_enabled(d.get(f"/RealOutputs/Swerve/{c}/driveErrorMetersPerSec", []), ep)

        if not curr:
            print(f"\n  [{c}] No data")
            continue

        _, curr_vals = to_arrays(curr)
        force_vals = np.array([current_to_wheel_force(i) for i in curr_vals])

        # How often force exceeds traction
        slipping = force_vals > MAX_TRACTION_FORCE
        pct_slip = 100 * np.sum(slipping) / len(slipping)

        # How much excess force (indicates severity)
        excess = np.where(slipping, force_vals - MAX_TRACTION_FORCE, 0)

        print(f"\n  [{c}]")
        print(f"    Stator current:  mean={np.mean(np.abs(curr_vals)):.1f}A  p95={np.percentile(np.abs(curr_vals), 95):.1f}A  max={np.max(np.abs(curr_vals)):.1f}A")
        print(f"    Wheel force:     mean={np.mean(force_vals):.1f}N  p95={np.percentile(force_vals, 95):.1f}N  max={np.max(force_vals):.1f}N")
        print(f"    Traction limit:  {MAX_TRACTION_FORCE:.1f}N")
        print(f"    Time over traction limit: {pct_slip:.1f}%")
        if np.sum(slipping) > 0:
            print(f"    Excess force when slipping: mean={np.mean(excess[slipping]):.1f}N  max={np.max(excess):.1f}N")

        # Correlate: velocity error during slip vs no-slip
        if curr and err:
            t, cur, er = align_by_time(curr, err)
            if len(t) > 50:
                forces = np.array([current_to_wheel_force(i) for i in cur])
                is_slip = forces > MAX_TRACTION_FORCE
                not_slip = ~is_slip
                if np.sum(is_slip) > 10 and np.sum(not_slip) > 10:
                    err_slip = np.mean(np.abs(er[is_slip]))
                    err_no_slip = np.mean(np.abs(er[not_slip]))
                    print(f"    |vel error| during slip:    {err_slip:.3f} m/s")
                    print(f"    |vel error| without slip:   {err_no_slip:.3f} m/s")
                    print(f"    Ratio: {err_slip / max(err_no_slip, 0.001):.1f}x")
                    if err_slip > err_no_slip * 1.5:
                        print(f"    ** SLIP IS CAUSING VELOCITY TRACKING ERROR **")

        # Acceleration spikes: compute dv/dt and check if high accel correlates with slip
        if vel and len(vel) > 10:
            t_v, v_v = to_arrays(vel)
            dt = np.diff(t_v)
            dv = np.diff(v_v)
            valid = dt > 0.005  # filter out duplicate timestamps
            if np.sum(valid) > 10:
                accel = dv[valid] / dt[valid]
                accel_t = t_v[1:][valid]
                high_accel = np.abs(accel) > 15  # m/s^2, very high
                if np.sum(high_accel) > 0:
                    print(f"    High accel (>15 m/s^2) events: {np.sum(high_accel)} ({100*np.sum(high_accel)/len(accel):.1f}%)")
                    print(f"    Peak accel: {np.max(np.abs(accel)):.1f} m/s^2")

        # Odometry impact: encoder distance during slip events
        if curr and pos:
            t_cp, cur_p, pos_p = align_by_time(curr, pos)
            if len(t_cp) > 50:
                forces_p = np.array([current_to_wheel_force(i) for i in cur_p])
                is_slip_p = forces_p > MAX_TRACTION_FORCE
                if np.sum(is_slip_p) > 1:
                    # Distance accumulated during slip
                    slip_mask = is_slip_p[:len(pos_p)]
                    slip_positions = pos_p[slip_mask]
                    slip_dist = np.sum(np.abs(np.diff(slip_positions))) if len(slip_positions) > 1 else 0
                    total_dist = np.sum(np.abs(np.diff(pos_p)))
                    if total_dist > 0:
                        print(f"    Encoder distance during slip: {slip_dist:.2f}m / {total_dist:.2f}m total ({100*slip_dist/total_dist:.1f}%)")

    # -- Recommended stator limit ---------------------------------------------
    print(f"\n  {'-'*80}")
    print(f"  RECOMMENDATION:")
    safe_current = MAX_TRACTION_FORCE * WHEEL_RAD / (KRAKEN_KT * DRIVE_RATIO) * 0.9  # 10% margin
    print(f"    Max current before slip: {slip_threshold_amps:.0f}A")
    print(f"    Recommended stator limit (10% margin): {safe_current:.0f}A")
    print(f"    Current stator limit: {STATOR_LIMIT:.0f}A")
    print()

def main():
    p = sys.argv[1] if len(sys.argv) > 1 else "../logs"
    if os.path.isfile(p):
        analyze(p)
    else:
        files = sorted([f for f in os.listdir(p) if f.endswith(".wpilog") and "_sim" not in f])
        if files:
            analyze(os.path.join(p, files[-1]))

if __name__ == "__main__":
    main()
