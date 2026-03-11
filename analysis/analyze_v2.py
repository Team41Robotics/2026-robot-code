"""Deep analysis of wpilog replay files - v2."""
import sys, math, os
from collections import defaultdict
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Generic readers
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

def read_int64s(reader, keys):
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
                    val = record.getInteger()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

def read_floats(reader, keys):
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
                    val = record.getFloat()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def stats(series):
    if not series:
        return None
    vals = [v for _, v in series]
    n = len(vals)
    mn = min(vals)
    mx = max(vals)
    mean = sum(vals) / n
    std = (sum((v - mean)**2 for v in vals) / n) ** 0.5
    return {"n": n, "min": mn, "max": mx, "mean": mean, "std": std}

def percentile(series, p):
    vals = sorted([v for _, v in series])
    idx = int(len(vals) * p / 100)
    return vals[min(idx, len(vals)-1)]

def pct_above(series, threshold):
    if not series:
        return 0
    vals = [v for _, v in series]
    return 100.0 * sum(1 for v in vals if v > threshold) / len(vals)

def pct_below(series, threshold):
    if not series:
        return 0
    vals = [v for _, v in series]
    return 100.0 * sum(1 for v in vals if v < threshold) / len(vals)

def filter_time_range(series, t_start, t_end):
    return [(t, v) for t, v in series if t_start <= t <= t_end]

def find_enabled_periods(enabled_series):
    """Return list of (start, end) for enabled periods."""
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

def find_auto_teleop_periods(enabled, auto_bool):
    """Return (auto_periods, teleop_periods) as lists of (start, end)."""
    en_periods = find_enabled_periods(enabled)
    auto_times = set()
    for t, v in auto_bool:
        if v:
            auto_times.add(round(t, 2))

    auto_periods = []
    teleop_periods = []
    for start, end in en_periods:
        # Check if this period overlaps with auto
        is_auto = any(round(t, 2) in auto_times for t, _ in auto_bool if start <= t <= end and _)
        if is_auto:
            auto_periods.append((start, end))
        else:
            teleop_periods.append((start, end))
    return auto_periods, teleop_periods

# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyze_file(path):
    print(f"\n{'='*90}")
    print(f"  ANALYZING: {os.path.basename(path)}")
    print(f"{'='*90}")

    # === PASS 1: System/timing/enabled state ===
    sys_double_keys = {
        "RealOutputs/LoggedRobot/FullCycleMS",
        "RealOutputs/LoggedRobot/UserCodeMS",
        "/SystemStats/BatteryVoltage",
        "/SystemStats/BatteryCurrent",
        "/SystemStats/CPUTempCelsius",
        "/SystemStats/3v3Rail/Voltage",
        "/SystemStats/5vRail/Voltage",
        "/SystemStats/6vRail/Voltage",
        "/PowerDistribution/TotalCurrent",
        "/PowerDistribution/Voltage",
        "/PowerDistribution/Temperature",
        "/DriverStation/MatchTime",
        "RealOutputs/LoggedRobot/GCTimeMS",
    }
    sys_bool_keys = {
        "/DriverStation/Enabled",
        "/DriverStation/Autonomous",
        "/SystemStats/BrownedOut",
        "/SystemStats/SystemActive",
    }
    sys_int_keys = {
        "/SystemStats/CANBus/OffCount",
        "/SystemStats/CANBus/ReceiveErrorCount",
        "/SystemStats/CANBus/TransmitErrorCount",
        "/SystemStats/CANBus/TxFullCount",
        "/RealOutputs//GC/TotalCollections",
        "/RealOutputs//GC/CollectionsSinceLast",
        "/RealOutputs//GC/CollectionTimeMSSinceLast",
        "/RealOutputs//GC/TotalCollectionTimeMS",
    }
    sys_float_keys = {
        "/SystemStats/CANBus/Utilization",
    }
    reader = DataLogReader(path)
    sys_d = read_doubles(reader, sys_double_keys)
    reader = DataLogReader(path)
    sys_b = read_booleans(reader, sys_bool_keys)
    reader = DataLogReader(path)
    sys_i = read_int64s(reader, sys_int_keys)
    reader = DataLogReader(path)
    sys_f = read_floats(reader, sys_float_keys)

    enabled = sys_b.get("/DriverStation/Enabled", [])
    auto = sys_b.get("/DriverStation/Autonomous", [])

    enabled_periods = find_enabled_periods(enabled)
    total_enabled_time = sum(e - s for s, e in enabled_periods)

    if enabled_periods:
        match_start = enabled_periods[0][0]
        match_end = enabled_periods[-1][1]
        print(f"\n  Match window: {match_start:.1f}s to {match_end:.1f}s (enabled {total_enabled_time:.1f}s)")
    else:
        match_start = 0
        match_end = float('inf')
        print("\n  WARNING: No enabled periods found!")

    # --- Loop timing ---
    cycle = sys_d.get("RealOutputs/LoggedRobot/FullCycleMS", [])
    user = sys_d.get("RealOutputs/LoggedRobot/UserCodeMS", [])

    # Filter to enabled time only
    cycle_en = []
    user_en = []
    for s, e in enabled_periods:
        cycle_en.extend(filter_time_range(cycle, s, e))
        user_en.extend(filter_time_range(user, s, e))

    print("\n--- LOOP TIMING (enabled only) ---")
    if cycle_en:
        s = stats(cycle_en)
        p95 = percentile(cycle_en, 95)
        p99 = percentile(cycle_en, 99)
        print(f"  Full cycle:  mean={s['mean']:.2f}ms  p95={p95:.2f}ms  p99={p99:.2f}ms  max={s['max']:.2f}ms")
        overruns_20 = pct_above(cycle_en, 20.0)
        overruns_40 = pct_above(cycle_en, 40.0)
        overruns_100 = pct_above(cycle_en, 100.0)
        print(f"  Overruns: >20ms={overruns_20:.1f}%  >40ms={overruns_40:.1f}%  >100ms={overruns_100:.1f}%")
        if overruns_40 > 1:
            print(f"  ** SEVERE: {overruns_40:.1f}% of loops >40ms — commands will skip cycles **")
    if user_en:
        s = stats(user_en)
        print(f"  User code:   mean={s['mean']:.2f}ms  max={s['max']:.2f}ms")

    # --- GC ---
    gc_time = sys_d.get("RealOutputs/LoggedRobot/GCTimeMS", [])
    gc_total_collections = sys_i.get("/RealOutputs//GC/TotalCollections", [])
    gc_since_last = sys_i.get("/RealOutputs//GC/CollectionTimeMSSinceLast", [])

    print("\n--- GARBAGE COLLECTION ---")
    if gc_total_collections:
        total_gc = gc_total_collections[-1][1] - gc_total_collections[0][1]
        print(f"  Total GC collections during log: {total_gc}")
    if gc_since_last:
        gc_pauses = [(t, v) for t, v in gc_since_last if v > 0]
        if gc_pauses:
            gs = stats(gc_pauses)
            print(f"  GC pauses: {gs['n']} events, mean={gs['mean']:.1f}ms, max={gs['max']:.1f}ms")
            big_pauses = [(t, v) for t, v in gc_pauses if v > 20]
            if big_pauses:
                print(f"  ** {len(big_pauses)} GC pauses >20ms — these cause loop overruns **")
                for t, v in big_pauses[:5]:
                    print(f"     t={t:.2f}s: {v}ms")
        else:
            print(f"  No GC pauses detected")

    # --- CAN bus ---
    can_util = sys_f.get("/SystemStats/CANBus/Utilization", [])
    can_off = sys_i.get("/SystemStats/CANBus/OffCount", [])
    can_rx_err = sys_i.get("/SystemStats/CANBus/ReceiveErrorCount", [])
    can_tx_err = sys_i.get("/SystemStats/CANBus/TransmitErrorCount", [])
    can_tx_full = sys_i.get("/SystemStats/CANBus/TxFullCount", [])

    print("\n--- CAN BUS ---")
    if can_util:
        cs = stats(can_util)
        print(f"  Utilization: mean={cs['mean']*100:.1f}%  max={cs['max']*100:.1f}%")
        if cs['max'] > 0.7:
            print(f"  ** HIGH CAN UTILIZATION ({cs['max']*100:.0f}%) — may cause dropped frames **")
    if can_off:
        off_total = can_off[-1][1] - can_off[0][1]
        if off_total > 0:
            print(f"  ** CAN BUS OFF events: {off_total} **")
    if can_rx_err:
        rx_total = can_rx_err[-1][1] - can_rx_err[0][1]
        if rx_total > 0:
            print(f"  CAN RX errors: {rx_total}")
    if can_tx_err:
        tx_total = can_tx_err[-1][1] - can_tx_err[0][1]
        if tx_total > 0:
            print(f"  CAN TX errors: {tx_total}")
    if can_tx_full:
        full_total = can_tx_full[-1][1] - can_tx_full[0][1]
        if full_total > 0:
            print(f"  ** CAN TX buffer full: {full_total} times — messages dropped **")

    # --- Battery/Power ---
    batt = sys_d.get("/SystemStats/BatteryVoltage", [])
    batt_curr = sys_d.get("/SystemStats/BatteryCurrent", [])
    pdp_curr = sys_d.get("/PowerDistribution/TotalCurrent", [])
    pdp_v = sys_d.get("/PowerDistribution/Voltage", [])
    pdp_temp = sys_d.get("/PowerDistribution/Temperature", [])
    browned_out = sys_b.get("/SystemStats/BrownedOut", [])
    cpu_temp = sys_d.get("/SystemStats/CPUTempCelsius", [])
    rail_5v = sys_d.get("/SystemStats/5vRail/Voltage", [])

    # Filter to enabled time
    batt_en = []
    pdp_curr_en = []
    for s_t, e_t in enabled_periods:
        batt_en.extend(filter_time_range(batt, s_t, e_t))
        pdp_curr_en.extend(filter_time_range(pdp_curr, s_t, e_t))

    print("\n--- POWER (enabled only) ---")
    if batt_en:
        bs = stats(batt_en)
        print(f"  Battery: min={bs['min']:.2f}V  mean={bs['mean']:.2f}V  max={bs['max']:.2f}V")
        if bs['min'] < 7.0:
            print(f"  ** BROWNOUT: voltage dropped to {bs['min']:.2f}V **")
            # Find when brownouts happened
            low_times = [(t, v) for t, v in batt_en if v < 7.0]
            if low_times:
                print(f"     First brownout at t={low_times[0][0]:.2f}s, last at t={low_times[-1][0]:.2f}s")
        elif bs['min'] < 9.0:
            print(f"  ** LOW VOLTAGE WARNING: min={bs['min']:.2f}V **")
    if browned_out:
        brownout_events = [(t, v) for t, v in browned_out if v]
        if brownout_events:
            print(f"  ** RoboRIO brownout flag triggered {len(brownout_events)} times **")

    if pdp_curr_en:
        ps = stats(pdp_curr_en)
        p95 = percentile(pdp_curr_en, 95)
        print(f"  Total current: mean={ps['mean']:.1f}A  p95={p95:.1f}A  max={ps['max']:.1f}A")
        if ps['max'] > 200:
            print(f"  ** EXTREME CURRENT ({ps['max']:.0f}A) — battery/breaker stress **")

    if rail_5v:
        r5s = stats(rail_5v)
        if r5s['min'] < 4.5:
            print(f"  ** 5V RAIL SAG: min={r5s['min']:.2f}V (cameras/sensors may glitch) **")

    if cpu_temp:
        ct = stats(cpu_temp)
        print(f"  CPU temp: mean={ct['mean']:.1f}C  max={ct['max']:.1f}C")
        if ct['max'] > 70:
            print(f"  ** HIGH CPU TEMP ({ct['max']:.0f}C) **")
    if pdp_temp:
        pt = stats(pdp_temp)
        if pt['max'] > 50:
            print(f"  ** HIGH PDH TEMP ({pt['max']:.0f}C) **")

    # --- Voltage-current correlation (internal resistance estimate) ---
    if batt_en and pdp_curr_en and len(batt_en) > 100:
        # Sample at common timestamps
        batt_dict = {round(t, 2): v for t, v in batt_en}
        pairs = []
        for t, curr in pdp_curr_en:
            v = batt_dict.get(round(t, 2))
            if v is not None and curr > 10:
                pairs.append((curr, v))
        if len(pairs) > 50:
            # Simple linear regression to estimate internal resistance
            n = len(pairs)
            sum_x = sum(c for c, _ in pairs)
            sum_y = sum(v for _, v in pairs)
            sum_xy = sum(c*v for c, v in pairs)
            sum_x2 = sum(c*c for c, _ in pairs)
            denom = n * sum_x2 - sum_x**2
            if denom != 0:
                slope = (n * sum_xy - sum_x * sum_y) / denom
                r_int = -slope  # V = V_oc - I*R_int, so slope is -R_int
                print(f"  Est. battery internal resistance: {r_int*1000:.0f} mΩ")
                if r_int > 0.025:
                    print(f"  ** HIGH INTERNAL RESISTANCE — battery may be old/weak **")

    # === PASS 2: Swerve deep dive ===
    swerve_keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        for k in ["driveErrorMetersPerSec", "turnErrorRadians", "driveFFVolts", "turnFFVolts",
                   "setpointVelMetersPerSec", "targetVelMetersPerSec", "setpointAngRadians",
                   "targetAngRadians", "setpointAngVelRadiansPerSec"]:
            swerve_keys.add(f"/RealOutputs/Swerve/{corner}/{k}")
        for k in ["DriveVelMetersPerSec", "DriveCurrentAmps", "DrivePosMeters",
                   "TurnCurrentAmps", "TurnPosRadians", "TurnAbsPosRadians",
                   "DriveVoltageVolts", "TurnVoltageVolts",
                   "DriveBusVoltageVolts", "TurnBusVoltageVolts"]:
            swerve_keys.add(f"/Swerve/{corner}/{k}")
    swerve_keys.add("/RealOutputs//Swerve/realSpeedMetersPerSecond")

    reader = DataLogReader(path)
    sw = read_doubles(reader, swerve_keys)

    print("\n--- SWERVE DRIVE (enabled only) ---")

    for corner in ["NE", "NW", "SE", "SW"]:
        de_all = sw.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        te_all = sw.get(f"/RealOutputs/Swerve/{corner}/turnErrorRadians", [])
        tv_all = sw.get(f"/RealOutputs/Swerve/{corner}/targetVelMetersPerSec", [])
        av_all = sw.get(f"/Swerve/{corner}/DriveVelMetersPerSec", [])
        dc_all = sw.get(f"/Swerve/{corner}/DriveCurrentAmps", [])
        tc_all = sw.get(f"/Swerve/{corner}/TurnCurrentAmps", [])
        dbv_all = sw.get(f"/Swerve/{corner}/DriveBusVoltageVolts", [])

        # Filter to enabled
        de = []
        te = []
        tv = []
        dc = []
        tc = []
        for s_t, e_t in enabled_periods:
            de.extend(filter_time_range(de_all, s_t, e_t))
            te.extend(filter_time_range(te_all, s_t, e_t))
            tv.extend(filter_time_range(tv_all, s_t, e_t))
            dc.extend(filter_time_range(dc_all, s_t, e_t))
            tc.extend(filter_time_range(tc_all, s_t, e_t))

        print(f"\n  [{corner}]")
        if de:
            # Separate into low-speed and high-speed
            de_abs = [(t, abs(v)) for t, v in de]
            ds = stats(de_abs)
            p95 = percentile(de_abs, 95)
            print(f"    Drive error (abs): mean={ds['mean']:.3f} m/s  p95={p95:.3f}  max={ds['max']:.3f}")

            # When target > 1 m/s, what's the tracking error?
            tv_dict = {round(t, 2): v for t, v in tv}
            high_speed_err = [(t, abs(v)) for t, v in de if abs(tv_dict.get(round(t, 2), 0)) > 1.0]
            if high_speed_err:
                hs = stats(high_speed_err)
                print(f"    Drive error @ >1m/s target: mean={hs['mean']:.3f}  max={hs['max']:.3f}")
                if hs['mean'] > 0.5:
                    print(f"    ** HIGH SPEED TRACKING POOR — check DRIVE_kP={4} or FF **")

        if te:
            te_abs = [(t, abs(v)) for t, v in te]
            ts_ = stats(te_abs)
            p95 = percentile(te_abs, 95)
            print(f"    Turn error (abs):  mean={math.degrees(ts_['mean']):.2f}deg  p95={math.degrees(p95):.2f}deg  max={math.degrees(ts_['max']):.2f}deg")
            if ts_['mean'] > 0.05:  # ~3 degrees
                print(f"    ** TURN ALIGNMENT POOR — mean error {math.degrees(ts_['mean']):.1f}deg **")

        if dc:
            dcs = stats(dc)
            print(f"    Drive current: mean={dcs['mean']:.1f}A  max={dcs['max']:.1f}A")
        if tc:
            tcs = stats(tc)
            print(f"    Turn current:  mean={tcs['mean']:.1f}A  max={tcs['max']:.1f}A")

    # Cross-module comparison
    print("\n  Cross-module comparison (mean drive error):")
    errors_by_module = {}
    for corner in ["NE", "NW", "SE", "SW"]:
        de_all = sw.get(f"/RealOutputs/Swerve/{corner}/driveErrorMetersPerSec", [])
        de = []
        for s_t, e_t in enabled_periods:
            de.extend(filter_time_range(de_all, s_t, e_t))
        if de:
            de_abs = [abs(v) for _, v in de]
            errors_by_module[corner] = sum(de_abs) / len(de_abs)
    if errors_by_module:
        mean_all = sum(errors_by_module.values()) / len(errors_by_module)
        for c, err in errors_by_module.items():
            flag = " ** OUTLIER **" if err > mean_all * 1.5 else ""
            print(f"    {c}: {err:.4f} m/s{flag}")

    # === PASS 3: Shooter deep dive ===
    shooter_keys = {
        "/RealOutputs//Shooter/targetFlywheelRPM",
        "/RealOutputs//Shooter/targetTurretPosRadians",
        "/RealOutputs//Shooter/turretProfilePosRadians",
        "/RealOutputs//Shooter/targetHoodPosRadians",
        "/RealOutputs//Shooter/hoodProfilePosRadians",
        "/RealOutputs//Shooter/flywheelErrorRPM",
        "/RealOutputs//Shooter/turretErrorRadians",
        "/RealOutputs//Shooter/hoodErrorRadians",
        "/RealOutputs//Shooter/turretProfileVelRadiansPerSec",
        "/RealOutputs//Shooter/hoodProfileVelRadiansPerSec",
        "//Shooter/FlywheelVelocityRPM",
        "//Shooter/TurretPosRadians",
        "//Shooter/HoodPosRadians",
        "//Shooter/TurretVelRadiansPerSec",
        "//Shooter/HoodVelRadiansPerSec",
        "//Shooter/FlywheelCurrentAmps",
        "//Shooter/TurretCurrentAmps",
        "//Shooter/HoodCurrentAmps",
        "//Shooter/FlywheelVoltageVolts",
        "//Shooter/TurretVoltageVolts",
        "//Shooter/HoodVoltageVolts",
        "//Shooter/FlywheelBusVoltageVolts",
    }
    shooter_bool_keys = {
        "//Shooter/TurretLimitSwitchOn",
        "//Shooter/IsHoodLimitSwitchOn",
    }
    reader = DataLogReader(path)
    sh = read_doubles(reader, shooter_keys)
    reader = DataLogReader(path)
    sh_b = read_booleans(reader, shooter_bool_keys)

    print("\n--- SHOOTER ---")

    fw_target = sh.get("/RealOutputs//Shooter/targetFlywheelRPM", [])
    fw_actual = sh.get("//Shooter/FlywheelVelocityRPM", [])
    fw_err = sh.get("/RealOutputs//Shooter/flywheelErrorRPM", [])
    turret_err = sh.get("/RealOutputs//Shooter/turretErrorRadians", [])
    hood_err = sh.get("/RealOutputs//Shooter/hoodErrorRadians", [])
    turret_pos = sh.get("//Shooter/TurretPosRadians", [])
    hood_pos = sh.get("//Shooter/HoodPosRadians", [])
    turret_target = sh.get("/RealOutputs//Shooter/targetTurretPosRadians", [])
    hood_target = sh.get("/RealOutputs//Shooter/targetHoodPosRadians", [])
    fw_bus_v = sh.get("//Shooter/FlywheelBusVoltageVolts", [])
    fw_curr = sh.get("//Shooter/FlywheelCurrentAmps", [])
    fw_voltage = sh.get("//Shooter/FlywheelVoltageVolts", [])
    turret_curr = sh.get("//Shooter/TurretCurrentAmps", [])
    hood_curr = sh.get("//Shooter/HoodCurrentAmps", [])
    turret_lim = sh_b.get("//Shooter/TurretLimitSwitchOn", [])

    # Flywheel spinup analysis
    if fw_target and fw_actual:
        # Find spinup events: target goes from 0 to >500
        target_dict = {round(t, 3): v for t, v in fw_target}
        actual_dict = {round(t, 3): v for t, v in fw_actual}

        # Identify shooting periods (target > 500 RPM)
        shooting_periods = []
        in_shooting = False
        shoot_start = 0
        for t, v in fw_target:
            if v > 500 and not in_shooting:
                in_shooting = True
                shoot_start = t
            elif v <= 500 and in_shooting:
                in_shooting = False
                shooting_periods.append((shoot_start, t))
        if in_shooting:
            shooting_periods.append((shoot_start, fw_target[-1][0]))

        total_shoot_time = sum(e - s for s, e in shooting_periods)
        print(f"  Shooting periods: {len(shooting_periods)}, total {total_shoot_time:.1f}s")

        # Spinup time analysis
        spinup_times = []
        for sp_start, sp_end in shooting_periods:
            # Find first target value in this period
            targets_in = [(t, v) for t, v in fw_target if sp_start <= t <= sp_end and v > 500]
            if not targets_in:
                continue
            tgt_rpm = targets_in[0][1]
            # Find when actual first reaches within 100 RPM of target
            for t, v in fw_actual:
                if t >= sp_start and abs(v - tgt_rpm) < 100:
                    spinup_times.append(t - sp_start)
                    break

        if spinup_times:
            mean_spinup = sum(spinup_times) / len(spinup_times)
            max_spinup = max(spinup_times)
            print(f"  Flywheel spinup: mean={mean_spinup:.2f}s  max={max_spinup:.2f}s ({len(spinup_times)} events)")
            if mean_spinup > 2.0:
                print(f"  ** SLOW SPINUP — consider increasing FLYWHEEL_kP or pre-spinning **")

        # Tracking during shooting
        shooting_errs = []
        for sp_start, sp_end in shooting_periods:
            for t, v in fw_err:
                if sp_start + 1.0 <= t <= sp_end:  # Skip first second (spinup)
                    shooting_errs.append((t, abs(v)))
        if shooting_errs:
            se = stats(shooting_errs)
            pct_100 = 100 * sum(1 for _, v in shooting_errs if v < 100) / len(shooting_errs)
            pct_50 = 100 * sum(1 for _, v in shooting_errs if v < 50) / len(shooting_errs)
            print(f"  Flywheel tracking (after spinup): mean err={se['mean']:.0f}RPM  max={se['max']:.0f}RPM")
            print(f"  Within 100RPM: {pct_100:.1f}%  Within 50RPM: {pct_50:.1f}%")
            if pct_100 < 80:
                print(f"  ** FLYWHEEL TRACKING POOR — tune FLYWHEEL_kP (currently 0.17662) **")

        # Flywheel current during shooting
        shooting_curr = []
        for sp_start, sp_end in shooting_periods:
            for t, v in fw_curr:
                if sp_start <= t <= sp_end:
                    shooting_curr.append((t, v))
        if shooting_curr:
            sc = stats(shooting_curr)
            print(f"  Flywheel current (shooting): mean={sc['mean']:.1f}A  max={sc['max']:.1f}A")

        # Flywheel bus voltage during shooting (voltage sag)
        if fw_bus_v:
            shooting_bv = []
            for sp_start, sp_end in shooting_periods:
                for t, v in fw_bus_v:
                    if sp_start <= t <= sp_end:
                        shooting_bv.append((t, v))
            if shooting_bv:
                bvs = stats(shooting_bv)
                print(f"  Bus voltage (shooting): min={bvs['min']:.2f}V  mean={bvs['mean']:.2f}V")
                if bvs['min'] < 8.0:
                    print(f"  ** SEVERE VOLTAGE SAG during shooting ({bvs['min']:.1f}V) **")

    # Turret tracking
    if turret_err:
        turret_err_en = []
        for s_t, e_t in enabled_periods:
            turret_err_en.extend(filter_time_range(turret_err, s_t, e_t))
        if turret_err_en:
            te_abs = [(t, abs(v)) for t, v in turret_err_en]
            tes = stats(te_abs)
            pct_2deg = 100 * sum(1 for _, v in te_abs if v < math.radians(2)) / len(te_abs)
            print(f"  Turret error: mean={math.degrees(tes['mean']):.2f}deg  max={math.degrees(tes['max']):.2f}deg  within 2deg: {pct_2deg:.1f}%")

    if hood_err:
        hood_err_en = []
        for s_t, e_t in enabled_periods:
            hood_err_en.extend(filter_time_range(hood_err, s_t, e_t))
        if hood_err_en:
            he_abs = [(t, abs(v)) for t, v in hood_err_en]
            hes = stats(he_abs)
            pct_2deg = 100 * sum(1 for _, v in he_abs if v < math.radians(2)) / len(he_abs)
            print(f"  Hood error:   mean={math.degrees(hes['mean']):.2f}deg  max={math.degrees(hes['max']):.2f}deg  within 2deg: {pct_2deg:.1f}%")

    # Turret limit switch events
    if turret_lim:
        lim_on = [(t, v) for t, v in turret_lim if v]
        print(f"  Turret limit switch triggers: {len(lim_on)}")

    # Turret position range
    if turret_pos:
        tp_en = []
        for s_t, e_t in enabled_periods:
            tp_en.extend(filter_time_range(turret_pos, s_t, e_t))
        if tp_en:
            tps = stats(tp_en)
            print(f"  Turret range: {math.degrees(tps['min']):.1f}deg to {math.degrees(tps['max']):.1f}deg")
            # Check if turret hit limits
            TURRET_MIN = math.degrees(-2.029)
            TURRET_MAX = 90  # PI/2
            if math.degrees(tps['min']) < TURRET_MIN + 2:
                print(f"  ** Turret near MIN limit ({math.degrees(tps['min']):.1f}deg, limit={TURRET_MIN:.1f}deg) **")
            if math.degrees(tps['max']) > TURRET_MAX - 2:
                print(f"  ** Turret near MAX limit ({math.degrees(tps['max']):.1f}deg, limit={TURRET_MAX:.1f}deg) **")

    # Hood range
    if hood_pos:
        hp_en = []
        for s_t, e_t in enabled_periods:
            hp_en.extend(filter_time_range(hood_pos, s_t, e_t))
        if hp_en:
            hps = stats(hp_en)
            print(f"  Hood range: {math.degrees(hps['min']):.1f}deg to {math.degrees(hps['max']):.1f}deg")

    # === PASS 4: Vision ===
    vision_str_keys = {
        "/RealOutputs//Vision/DuckyNE/method",
        "/RealOutputs//Vision/DuckySE/method",
    }
    vision_int_keys = {
        "/RealOutputs//Vision/DuckyNE/nTargets",
        "/RealOutputs//Vision/DuckySE/nTargets",
    }
    vision_bool_keys = {
        "//Vision/DuckyNE/IsConnected",
        "//Vision/DuckySE/IsConnected",
        "/RealOutputs//Vision/enabled",
    }
    reader = DataLogReader(path)
    vis_s = read_strings(reader, vision_str_keys)
    reader = DataLogReader(path)
    vis_i = read_int64s(reader, vision_int_keys)
    reader = DataLogReader(path)
    vis_b = read_booleans(reader, vision_bool_keys)

    print("\n--- VISION ---")
    vision_enabled = vis_b.get("/RealOutputs//Vision/enabled", [])
    if vision_enabled:
        disabled_count = sum(1 for _, v in vision_enabled if not v)
        if disabled_count > 0:
            print(f"  Vision disabled for {disabled_count} samples")

    for cam in ["DuckyNE", "DuckySE"]:
        methods = vis_s.get(f"/RealOutputs//Vision/{cam}/method", [])
        connected = vis_b.get(f"//Vision/{cam}/IsConnected", [])
        ntargets = vis_i.get(f"/RealOutputs//Vision/{cam}/nTargets", [])

        print(f"\n  [{cam}]")
        if connected:
            disconnects = []
            for i in range(1, len(connected)):
                if connected[i-1][1] and not connected[i][1]:
                    disconnects.append(connected[i][0])
            if disconnects:
                print(f"    ** {len(disconnects)} disconnect events **")
                for t in disconnects[:3]:
                    print(f"       t={t:.2f}s")
            pct_connected = 100 * sum(1 for _, v in connected if v) / len(connected)
            print(f"    Connected: {pct_connected:.1f}%")

        if methods:
            # Filter to enabled time
            methods_en = []
            for s_t, e_t in enabled_periods:
                methods_en.extend([(t, v) for t, v in methods if s_t <= t <= e_t])

            counts = defaultdict(int)
            for _, m in methods_en:
                counts[m] += 1
            total = len(methods_en)
            if total > 0:
                print(f"    {total} vision updates during match:")
                for m, c in sorted(counts.items(), key=lambda x: -x[1]):
                    print(f"      {m}: {c} ({100*c/total:.1f}%)")

                useful = total - counts.get("none", 0)
                print(f"    Useful updates: {useful} ({100*useful/total:.1f}%)")

        if ntargets:
            nt_en = []
            for s_t, e_t in enabled_periods:
                nt_en.extend(filter_time_range(ntargets, s_t, e_t))
            if nt_en:
                multi_tag = sum(1 for _, v in nt_en if v >= 2)
                single_tag = sum(1 for _, v in nt_en if v == 1)
                no_tag = sum(1 for _, v in nt_en if v == 0)
                print(f"    Tags seen: 0={no_tag}  1={single_tag}  2+={multi_tag}")

    # === PASS 5: Odometry / Pose ===
    odom_keys = {
        "/RealOutputs//Odom/xMeters",
        "/RealOutputs//Odom/yMeters",
        "/RealOutputs//Odom/rotRadians",
        "/RealOutputs//Odom/imuYawRadians",
    }
    reader = DataLogReader(path)
    odom = read_doubles(reader, odom_keys)

    print("\n--- ODOMETRY ---")
    ox = odom.get("/RealOutputs//Odom/xMeters", [])
    oy = odom.get("/RealOutputs//Odom/yMeters", [])
    orot = odom.get("/RealOutputs//Odom/rotRadians", [])
    imu_yaw = odom.get("/RealOutputs//Odom/imuYawRadians", [])

    if ox and oy:
        # Filter to enabled
        ox_en = []
        oy_en = []
        for s_t, e_t in enabled_periods:
            ox_en.extend(filter_time_range(ox, s_t, e_t))
            oy_en.extend(filter_time_range(oy, s_t, e_t))

        if ox_en:
            sxe = stats(ox_en)
            sye = stats(oy_en)
            print(f"  X range: {sxe['min']:.2f} to {sxe['max']:.2f} m")
            print(f"  Y range: {sye['min']:.2f} to {sye['max']:.2f} m")

            # Check for out-of-field poses
            FIELD_LEN = 17.55  # approximate
            FIELD_WID = 8.05
            oob = sum(1 for _, v in ox_en if v < -0.5 or v > FIELD_LEN + 0.5) + \
                  sum(1 for _, v in oy_en if v < -0.5 or v > FIELD_WID + 0.5)
            if oob > 0:
                print(f"  ** {oob} out-of-field pose estimates **")

            # Pose jumps (vision corrections)
            jumps = []
            for i in range(1, len(ox_en)):
                dt = ox_en[i][0] - ox_en[i-1][0]
                if dt < 0.1:  # Only check consecutive frames
                    dx = abs(ox_en[i][1] - ox_en[i-1][1])
                    # Find matching Y
                    if i < len(oy_en):
                        dy = abs(oy_en[i][1] - oy_en[i-1][1])
                        dist = math.hypot(dx, dy)
                        if dist > 0.5:
                            jumps.append((ox_en[i][0], dist))
            if jumps:
                print(f"  Pose jumps (>0.5m): {len(jumps)}")
                big_jumps = [(t, d) for t, d in jumps if d > 1.0]
                if big_jumps:
                    print(f"  ** {len(big_jumps)} jumps >1.0m — vision covariance too trusting **")
                    for t, d in big_jumps[:5]:
                        print(f"     t={t:.2f}s: {d:.2f}m")

    # IMU yaw drift check (compare start/end when disabled)
    if imu_yaw and len(imu_yaw) > 10:
        # Check for discontinuities (IMU reset/calibration)
        imu_jumps = 0
        for i in range(1, len(imu_yaw)):
            if abs(imu_yaw[i][1] - imu_yaw[i-1][1]) > 0.5:  # >28 deg jump
                imu_jumps += 1
        if imu_jumps > 0:
            print(f"  ** IMU yaw jumps: {imu_jumps} (>28deg instantaneous change) **")

    # === PASS 6: Auto tracking ===
    auto_keys = {
        "/RealOutputs//Auto/xError",
        "/RealOutputs//Auto/yError",
    }
    reader = DataLogReader(path)
    auto_d = read_doubles(reader, auto_keys)

    ax_err = auto_d.get("/RealOutputs//Auto/xError", [])
    ay_err = auto_d.get("/RealOutputs//Auto/yError", [])

    if ax_err and ay_err:
        print("\n--- AUTO PATH TRACKING ---")
        axs = stats(ax_err)
        ays = stats(ay_err)
        print(f"  X error: mean={axs['mean']:.3f}m  max={axs['max']:.3f}m  std={axs['std']:.3f}m")
        print(f"  Y error: mean={ays['mean']:.3f}m  max={ays['max']:.3f}m  std={ays['std']:.3f}m")

        # Combined distance error
        combined = []
        ax_dict = {round(t, 3): v for t, v in ax_err}
        for t, yv in ay_err:
            xv = ax_dict.get(round(t, 3))
            if xv is not None:
                combined.append((t, math.hypot(xv, yv)))
        if combined:
            cs = stats(combined)
            pct_10cm = 100 * sum(1 for _, v in combined if v < 0.1) / len(combined)
            pct_25cm = 100 * sum(1 for _, v in combined if v < 0.25) / len(combined)
            print(f"  Distance error: mean={cs['mean']:.3f}m  max={cs['max']:.3f}m")
            print(f"  Within 10cm: {pct_10cm:.1f}%  Within 25cm: {pct_25cm:.1f}%")
            if cs['mean'] > 0.2:
                print(f"  ** AUTO TRACKING POOR — check odometry, PID, or path **")

    # === PASS 7: Indexer ===
    indexer_keys = {
        "/RealOutputs//Indexer/targetElevatorVoltage",
        "/RealOutputs//Indexer/targetSpinVoltage",
        "//Indexer/ElevatorCurrentAmps",
        "//Indexer/SpinCurrentAmps",
        "//Indexer/ElevatorVelocityRPM",
        "//Indexer/SpinVelocityRPM",
    }
    reader = DataLogReader(path)
    idx = read_doubles(reader, indexer_keys)

    elev_v = idx.get("/RealOutputs//Indexer/targetElevatorVoltage", [])
    spin_v = idx.get("/RealOutputs//Indexer/targetSpinVoltage", [])
    elev_c = idx.get("//Indexer/ElevatorCurrentAmps", [])
    spin_c = idx.get("//Indexer/SpinCurrentAmps", [])

    if elev_v or spin_v:
        print("\n--- INDEXER ---")
        if elev_v:
            evs = stats(elev_v)
            active_elev = [(t, v) for t, v in elev_v if abs(v) > 0.1]
            print(f"  Elevator active: {len(active_elev)}/{len(elev_v)} samples")
        if spin_v:
            svs = stats(spin_v)
            active_spin = [(t, v) for t, v in spin_v if abs(v) > 0.1]
            print(f"  Spin active: {len(active_spin)}/{len(spin_v)} samples")
        if elev_c:
            ecs = stats(elev_c)
            print(f"  Elevator current: mean={ecs['mean']:.1f}A  max={ecs['max']:.1f}A")
            if ecs['max'] > 30:
                print(f"  ** HIGH ELEVATOR CURRENT ({ecs['max']:.0f}A) — possible jam **")
        if spin_c:
            scs = stats(spin_c)
            print(f"  Spin current: mean={scs['mean']:.1f}A  max={scs['max']:.1f}A")

    # === PASS 8: IMU ===
    imu_keys = {
        "//IMU/YawRadians",
        "//IMU/PitchRadians",
        "//IMU/RollRadians",
    }
    imu_bool_keys = {
        "//IMU/IsConnected",
        "//IMU/IsCalibrating",
    }
    reader = DataLogReader(path)
    imu_d = read_doubles(reader, imu_keys)
    reader = DataLogReader(path)
    imu_b = read_booleans(reader, imu_bool_keys)

    imu_connected = imu_b.get("//IMU/IsConnected", [])
    imu_cal = imu_b.get("//IMU/IsCalibrating", [])

    print("\n--- IMU ---")
    if imu_connected:
        disconnected = sum(1 for _, v in imu_connected if not v)
        if disconnected > 0:
            print(f"  ** IMU disconnected for {disconnected} samples **")
    if imu_cal:
        calibrating = [(t, v) for t, v in imu_cal if v]
        if calibrating:
            print(f"  IMU calibrating: {len(calibrating)} samples")
            # Check if calibrating during enabled
            cal_during_enabled = False
            for t, _ in calibrating:
                for s_t, e_t in enabled_periods:
                    if s_t <= t <= e_t:
                        cal_during_enabled = True
                        break
            if cal_during_enabled:
                print(f"  ** IMU CALIBRATING DURING MATCH — heading unreliable **")

    pitch = imu_d.get("//IMU/PitchRadians", [])
    roll = imu_d.get("//IMU/RollRadians", [])
    if pitch:
        ps = stats(pitch)
        print(f"  Pitch: mean={math.degrees(ps['mean']):.1f}deg  max={math.degrees(ps['max']):.1f}deg")
    if roll:
        rs = stats(roll)
        print(f"  Roll:  mean={math.degrees(rs['mean']):.1f}deg  max={math.degrees(rs['max']):.1f}deg")

    # === PASS 9: Console output ===
    console_keys = {"/RealOutputs/Console"}
    reader = DataLogReader(path)
    console = read_strings(reader, console_keys)

    console_msgs = console.get("/RealOutputs/Console", [])
    if console_msgs:
        print(f"\n--- CONSOLE ({len(console_msgs)} messages) ---")
        # Look for errors/warnings
        errors = [(t, m) for t, m in console_msgs if any(w in m.lower() for w in ["error", "exception", "fail", "warn", "null"])]
        if errors:
            print(f"  Errors/warnings: {len(errors)}")
            seen = set()
            for t, m in errors[:20]:
                short = m.strip()[:120]
                if short not in seen:
                    seen.add(short)
                    print(f"    t={t:.2f}s: {short}")

    # === PASS 10: Climber ===
    climber_keys = {
        "/Climber/PosMeters",
        "/Climber/VelMetersPerSec",
        "/Climber/CurrentAmps",
        "/Climber/VoltageVolts",
        "/Climber/ActuatorCurrentAmps",
        "/RealOutputs/Climber/targetVoltage",
        "/RealOutputs/Climber/actuatorTargetVoltage",
    }
    climber_bool_keys = {
        "/Climber/LimitTop",
        "/Climber/LimitBottom",
        "/Climber/LimitTop2",
        "/Climber/LimitBottom2",
        "/Climber/LimitActuator",
        "/Climber/RetractSwitch",
    }
    reader = DataLogReader(path)
    cl = read_doubles(reader, climber_keys)
    reader = DataLogReader(path)
    cl_b = read_booleans(reader, climber_bool_keys)

    cl_tv = cl.get("/RealOutputs/Climber/targetVoltage", [])
    cl_curr = cl.get("/Climber/CurrentAmps", [])

    if cl_tv:
        active_climb = [(t, v) for t, v in cl_tv if abs(v) > 0.1]
        if active_climb:
            print(f"\n--- CLIMBER ---")
            print(f"  Active: {len(active_climb)} samples (from t={active_climb[0][0]:.1f}s to t={active_climb[-1][0]:.1f}s)")
            if cl_curr:
                climb_curr = []
                for t, v in cl_curr:
                    if any(abs(tv) > 0.1 for tt, tv in cl_tv if abs(tt - t) < 0.1):
                        climb_curr.append((t, v))
                if climb_curr:
                    ccs = stats(climb_curr)
                    print(f"  Climb current: mean={ccs['mean']:.1f}A  max={ccs['max']:.1f}A")

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
            import traceback
            print(f"  ERROR analyzing {f}: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()
