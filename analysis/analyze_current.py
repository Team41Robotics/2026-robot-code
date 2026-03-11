"""Per-motor current draw analysis from wpilog files.

CurrentAmps    = stator current  (torque / motor heat)
BusCurrentAmps = supply current  (battery draw, wiring / breaker load)

Usage:
    python analyze_current.py <path/to/file.wpilog>
    python analyze_current.py <path/to/logs/dir>
"""
import sys, os, math
from collections import defaultdict
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def read_doubles(path, keys):
    entries = {}
    data = defaultdict(list)
    reader = DataLogReader(path)
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


def read_double_arrays(path, keys):
    entries = {}
    data = defaultdict(list)
    reader = DataLogReader(path)
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in keys:
                entries[d.entry] = d.name
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entries:
                try:
                    data[entries[eid]].append((record.getTimestamp() / 1e6, record.getDoubleArray()))
                except Exception:
                    pass
    return data


def read_booleans(path, keys):
    entries = {}
    data = defaultdict(list)
    reader = DataLogReader(path)
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


def stats(series):
    if not series:
        return None
    vals = [v for _, v in series]
    n = len(vals)
    mean = sum(vals) / n
    vals_s = sorted(vals)
    p95 = vals_s[min(int(n * 0.95), n - 1)]
    p99 = vals_s[min(int(n * 0.99), n - 1)]
    return {"n": n, "mean": mean, "max": max(vals), "p95": p95, "p99": p99}


def filter_time(series, periods):
    out = []
    for s, e in periods:
        out.extend((t, v) for t, v in series if s <= t <= e)
    return out


def enabled_periods(enabled_series):
    periods, start = [], None
    for t, v in enabled_series:
        if v and start is None:
            start = t
        elif not v and start is not None:
            periods.append((start, t))
            start = None
    if start is not None and enabled_series:
        periods.append((start, enabled_series[-1][0]))
    return periods


def bar(mean, max_val, width=30):
    scale = width / max(max_val, 1)
    filled = int(mean * scale)
    return "[" + "█" * filled + "░" * (width - filled) + f"] {mean:5.1f}A (max {max_val:.0f}A)"


def print_motor(label, stator_series, supply_series, *, warn_stator=None, warn_supply=None, indent="  "):
    st = stats(stator_series) if stator_series else None
    su = stats(supply_series) if supply_series else None

    if not st and not su:
        print(f"{indent}{label:30s}  (no data)")
        return

    parts = []
    if st:
        flag = " ⚠ HIGH" if warn_stator and st["max"] > warn_stator else ""
        parts.append(f"stator mean={st['mean']:5.1f}A  p95={st['p95']:5.1f}A  max={st['max']:5.1f}A{flag}")
    if su:
        flag = " ⚠ HIGH" if warn_supply and su["max"] > warn_supply else ""
        parts.append(f"supply mean={su['mean']:5.1f}A  p95={su['p95']:5.1f}A  max={su['max']:5.1f}A{flag}")

    print(f"{indent}{label:30s}  " + "    |    ".join(parts))


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze(path):
    print(f"\n{'='*100}")
    print(f"  CURRENT ANALYSIS: {os.path.basename(path)}")
    print(f"{'='*100}")

    # --- Enabled periods ---
    bool_data = read_booleans(path, {"/DriverStation/Enabled", "/DriverStation/Autonomous"})
    en_periods = enabled_periods(bool_data.get("/DriverStation/Enabled", []))
    total_en = sum(e - s for s, e in en_periods)
    if not en_periods:
        print("  WARNING: no enabled periods found — showing full-log stats instead")
        en_periods = [(0, float("inf"))]
    else:
        print(f"  Enabled periods: {len(en_periods)}, total {total_en:.1f}s")
        for i, (s, e) in enumerate(en_periods):
            print(f"    Period {i+1}: {s:.1f}s – {e:.1f}s  ({e-s:.1f}s)")

    # --- Keys to read ---
    motor_keys = {
        # Swerve
        "/Swerve/NE/DriveCurrentAmps", "/Swerve/NE/DriveBusCurrentAmps",
        "/Swerve/NE/TurnCurrentAmps",  "/Swerve/NE/TurnBusCurrentAmps",
        "/Swerve/NW/DriveCurrentAmps", "/Swerve/NW/DriveBusCurrentAmps",
        "/Swerve/NW/TurnCurrentAmps",  "/Swerve/NW/TurnBusCurrentAmps",
        "/Swerve/SE/DriveCurrentAmps", "/Swerve/SE/DriveBusCurrentAmps",
        "/Swerve/SE/TurnCurrentAmps",  "/Swerve/SE/TurnBusCurrentAmps",
        "/Swerve/SW/DriveCurrentAmps", "/Swerve/SW/DriveBusCurrentAmps",
        "/Swerve/SW/TurnCurrentAmps",  "/Swerve/SW/TurnBusCurrentAmps",
        # Shooter
        "//Shooter/TurretCurrentAmps",  "//Shooter/TurretBusCurrentAmps",
        "//Shooter/HoodCurrentAmps",    "//Shooter/HoodBusCurrentAmps",
        "//Shooter/FlywheelCurrentAmps","//Shooter/FlywheelBusCurrentAmps",
        # Indexer
        "//Indexer/SpinCurrentAmps",    "//Indexer/SpinBusCurrentAmps",
        "//Indexer/ElevatorCurrentAmps","//Indexer/ElevatorBusCurrentAmps",
        # Intake
        "//Intake/IntakeCurrentAmps",   "//Intake/IntakeBusCurrentAmps",
        "//Intake/JointCurrentAmps",
        # Climber (may not exist)
        "/Climber/CurrentAmps",
        # System
        "/SystemStats/BatteryCurrent",
        "/PowerDistribution/TotalCurrent",
        "/PowerDistribution/Voltage",
    }

    d = read_doubles(path, motor_keys)
    pdh_ch = read_double_arrays(path, {"/PowerDistribution/ChannelCurrent"})

    def en(key):
        return filter_time(d.get(key, []), en_periods)

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  SYSTEM POWER")
    print(f"{'─'*100}")

    batt_curr = en("/SystemStats/BatteryCurrent")
    pdh_curr  = en("/PowerDistribution/TotalCurrent")
    pdh_volt  = en("/PowerDistribution/Voltage")

    if batt_curr:
        bs = stats(batt_curr)
        print(f"  Battery draw:  {bar(bs['mean'], bs['max'])}  p95={bs['p95']:.0f}A")
    if pdh_curr:
        ps = stats(pdh_curr)
        print(f"  PDH total:     {bar(ps['mean'], ps['max'])}  p95={ps['p95']:.0f}A")
    if pdh_volt:
        vs = stats(pdh_volt)
        print(f"  PDH voltage:   mean={vs['mean']:.2f}V  min={vs['mean'] - vs['p95']:.2f}V (approx)  actual_min check below")
        low = [(t, v) for t, v in pdh_volt if v < 9.0]
        if low:
            print(f"  ⚠  Voltage <9V for {len(low)} samples — first at t={low[0][0]:.1f}s, worst={min(v for _,v in low):.2f}V")

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  SWERVE  (CANivore — does NOT draw from RIO CAN bus)")
    print(f"{'─'*100}")
    print(f"  {'Motor':30s}  {'Stator (motor torque/heat)':40s}  Supply (battery draw)")

    drive_total_supply_mean = 0
    for corner in ["NE", "NW", "SE", "SW"]:
        dst = en(f"/Swerve/{corner}/DriveCurrentAmps")
        dsu = en(f"/Swerve/{corner}/DriveBusCurrentAmps")
        tst = en(f"/Swerve/{corner}/TurnCurrentAmps")
        tsu = en(f"/Swerve/{corner}/TurnBusCurrentAmps")
        print_motor(f"Drive {corner}", dst, dsu, warn_stator=90, warn_supply=50)
        print_motor(f"Turn  {corner}", tst, tsu, warn_stator=40, warn_supply=25)
        su_s = stats(dsu)
        if su_s:
            drive_total_supply_mean += su_s["mean"]

    print(f"\n  Estimated total swerve supply draw (mean): {drive_total_supply_mean:.1f}A  (×4 drive modules)")

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  SHOOTER  (RIO CAN bus)")
    print(f"{'─'*100}")
    print(f"  {'Motor':30s}  {'Stator (motor torque/heat)':40s}  Supply (battery draw)")

    # Detect flywheel shooting periods (target > 500 RPM)
    fw_target_keys = {"/RealOutputs//Shooter/targetFlywheelRPM"}
    fw_t = read_doubles(path, fw_target_keys)
    fw_target_series = filter_time(fw_t.get("/RealOutputs//Shooter/targetFlywheelRPM", []), en_periods)
    shoot_periods = []
    in_shoot, shoot_start = False, 0
    for t, v in fw_target_series:
        if v > 500 and not in_shoot:
            in_shoot, shoot_start = True, t
        elif v <= 500 and in_shoot:
            in_shoot = False
            shoot_periods.append((shoot_start, t))
    if in_shoot:
        shoot_periods.append((shoot_start, fw_target_series[-1][0]))

    print_motor("Turret",          en("//Shooter/TurretCurrentAmps"),   en("//Shooter/TurretBusCurrentAmps"),  warn_stator=60,  warn_supply=35)
    print_motor("Hood",            en("//Shooter/HoodCurrentAmps"),     en("//Shooter/HoodBusCurrentAmps"),    warn_stator=30,  warn_supply=15)
    print_motor("Flywheel (all)",  en("//Shooter/FlywheelCurrentAmps"), en("//Shooter/FlywheelBusCurrentAmps"),warn_stator=100, warn_supply=50)

    if shoot_periods:
        fw_st_shoot = filter_time(d.get("//Shooter/FlywheelCurrentAmps", []), shoot_periods)
        fw_su_shoot = filter_time(d.get("//Shooter/FlywheelBusCurrentAmps", []), shoot_periods)
        print_motor("  └─ during shooting", fw_st_shoot, fw_su_shoot, warn_stator=100, warn_supply=50, indent="    ")
        fw_idle_periods = [(s, e) for s, e in en_periods
                           if not any(sp_s <= s <= sp_e or s <= sp_s <= e for sp_s, sp_e in shoot_periods)]
        fw_st_idle = filter_time(d.get("//Shooter/FlywheelCurrentAmps", []), fw_idle_periods)
        fw_su_idle = filter_time(d.get("//Shooter/FlywheelBusCurrentAmps", []), fw_idle_periods)
        print_motor("  └─ idle/spinup",     fw_st_idle,  fw_su_idle,  warn_stator=60, warn_supply=40, indent="    ")

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  INDEXER  (RIO CAN bus)")
    print(f"{'─'*100}")
    print(f"  {'Motor':30s}  {'Stator (motor torque/heat)':40s}  Supply (battery draw)")

    print_motor("Elevator", en("//Indexer/ElevatorCurrentAmps"), en("//Indexer/ElevatorBusCurrentAmps"), warn_stator=50, warn_supply=35)
    print_motor("Spin",     en("//Indexer/SpinCurrentAmps"),     en("//Indexer/SpinBusCurrentAmps"),     warn_stator=35, warn_supply=25)

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  INTAKE  (RIO CAN bus)")
    print(f"{'─'*100}")
    print(f"  {'Motor':30s}  {'Stator (motor torque/heat)':40s}  Supply (battery draw)")

    print_motor("Intake roller (TalonFX)", en("//Intake/IntakeCurrentAmps"), en("//Intake/IntakeBusCurrentAmps"), warn_stator=80, warn_supply=35)
    print_motor("Joint (SparkMax)",        en("//Intake/JointCurrentAmps"),   None,                               warn_stator=35)

    # =========================================================================
    # Climber
    cl_curr = en("/Climber/CurrentAmps")
    if cl_curr and stats(cl_curr)["max"] > 1.0:
        print(f"\n{'─'*100}")
        print("  CLIMBER  (RIO CAN bus)")
        print(f"{'─'*100}")
        print_motor("Climber", cl_curr, None, warn_stator=80)

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  ESTIMATED CURRENT BUDGET (mean, enabled)")
    print(f"{'─'*100}")

    def mean_of(key):
        s = stats(en(key))
        return s["mean"] if s else 0.0

    # RIO bus motors (supply = battery draw)
    turret_supply  = mean_of("//Shooter/TurretBusCurrentAmps")
    hood_supply    = mean_of("//Shooter/HoodBusCurrentAmps")
    fw_supply      = mean_of("//Shooter/FlywheelBusCurrentAmps")
    spin_supply    = mean_of("//Indexer/SpinBusCurrentAmps")
    elev_supply    = mean_of("//Indexer/ElevatorBusCurrentAmps")
    intake_supply  = mean_of("//Intake/IntakeBusCurrentAmps")
    joint_curr     = mean_of("//Intake/JointCurrentAmps")  # SparkMax, no separate supply signal

    # Swerve (CANivore, but still draws from battery)
    swerve_supply = sum(
        mean_of(f"/Swerve/{c}/DriveBusCurrentAmps") + mean_of(f"/Swerve/{c}/TurnBusCurrentAmps")
        for c in ["NE", "NW", "SE", "SW"]
    )

    total_measured = turret_supply + hood_supply + fw_supply + spin_supply + elev_supply + intake_supply + joint_curr + swerve_supply
    pdh_mean = stats(en("/PowerDistribution/TotalCurrent"))
    pdh_mean_val = pdh_mean["mean"] if pdh_mean else 0

    rows = [
        ("Swerve (8 motors)",       swerve_supply),
        ("Flywheel (2 Krakens)",    fw_supply),
        ("Turret",                  turret_supply),
        ("Hood",                    hood_supply),
        ("Indexer elevator",        elev_supply),
        ("Indexer spin",            spin_supply),
        ("Intake roller",           intake_supply),
        ("Intake joint (SparkMax)", joint_curr),
    ]

    max_curr = max(v for _, v in rows) if rows else 1
    for label, curr in sorted(rows, key=lambda x: -x[1]):
        if curr > 0.1:
            pct = 100 * curr / max(total_measured, 1)
            bstr = bar(curr, max_curr, width=25)
            print(f"  {label:28s}  {bstr}  ({pct:.0f}% of motors)")

    print(f"\n  Measured motor total (supply): {total_measured:.1f}A")
    if pdh_mean_val > 0:
        overhead = pdh_mean_val - total_measured
        print(f"  PDH total (incl. electronics): {pdh_mean_val:.1f}A  (overhead/electronics: {max(overhead,0):.1f}A)")

    # =========================================================================
    print(f"\n{'─'*100}")
    print("  PEAK EVENTS — current spikes during enabled (top 10 moments by total draw)")
    print(f"{'─'*100}")

    if pdh_mean_val > 0:
        pdh_series = en("/PowerDistribution/TotalCurrent")
        sorted_peaks = sorted(pdh_series, key=lambda x: -x[1])
        seen_times = []
        print(f"  {'Time':>7s}  {'PDH Total':>10s}")
        for t, v in sorted_peaks:
            # Deduplicate within 0.5s
            if all(abs(t - st) > 0.5 for st in seen_times):
                print(f"  {t:7.2f}s  {v:8.1f}A")
                seen_times.append(t)
            if len(seen_times) >= 10:
                break

    print()


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "../logs"

    if os.path.isfile(arg):
        analyze(arg)
        return

    files = sorted(f for f in os.listdir(arg) if f.endswith(".wpilog") and "_sim" not in f)
    if not files:
        print(f"No .wpilog files found in {arg}")
        return

    print(f"Found {len(files)} log files")
    for f in files:
        try:
            analyze(os.path.join(arg, f))
        except Exception as e:
            import traceback
            print(f"  ERROR: {f}: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()
