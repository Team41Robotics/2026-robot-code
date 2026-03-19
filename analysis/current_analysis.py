"""Analyze drive and turn motor currents from match logs to evaluate current limits."""
from wpiutil.log import DataLogReader
import numpy as np
import glob as globmod
import os

LOG_DIR = 'c:/Users/Robotics41/Desktop/2026-robot-code-new/logs'
MODULES = ['NE', 'NW', 'SE']

SIGNALS = {
    'drive_stator': 'DriveCurrentAmps',
    'drive_supply': 'DriveBusCurrentAmps',
    'turn_stator': 'TurnCurrentAmps',
    'turn_supply': 'TurnBusCurrentAmps',
    'drive_volt': 'DriveVoltageVolts',
    'turn_volt': 'TurnVoltageVolts',
    'drive_vel': 'DriveVelMetersPerSec',
    'turn_vel': 'TurnVelRadiansPerSec',
}

def load_all(log_paths):
    all_data = {}
    for log_path in log_paths:
        reader = DataLogReader(log_path)
        entries = {}
        data = {}
        for record in reader:
            if record.isControl():
                if record.isStart():
                    s = record.getStartData()
                    entries[s.entry] = s.name
            else:
                eid = record.getEntry()
                name = entries.get(eid)
                if name is None:
                    continue
                for mod in MODULES:
                    for key, sig in SIGNALS.items():
                        if name == f'/Swerve/{mod}/{sig}':
                            data.setdefault((mod, key), []).append(
                                (record.getTimestamp()/1e6, record.getDouble()))
        logname = os.path.basename(log_path)
        all_data[logname] = data
    return all_data

if __name__ == '__main__':
    DAY1_LOGS = sorted([
        f for f in globmod.glob(os.path.join(LOG_DIR, 'akit_26-03-07_*.wpilog'))
        if '_sim' not in f
    ])
    print(f"Loading {len(DAY1_LOGS)} day-1 logs...")
    all_data = load_all(DAY1_LOGS)

    # ==========================================
    # DRIVE MOTOR CURRENTS
    # ==========================================
    print("=" * 70)
    print("DRIVE MOTOR CURRENTS")
    print("  Current limits: supply=60A (2s ramp), stator=70A")
    print("=" * 70)

    all_drive_stator = []
    all_drive_supply = []
    for logname, data in all_data.items():
        for mod in MODULES:
            stator = data.get((mod, 'drive_stator'), [])
            supply = data.get((mod, 'drive_supply'), [])
            if len(stator) < 100:
                continue
            s_arr = np.array(stator)[:, 1]
            all_drive_stator.extend(s_arr)
            if len(supply) >= 100:
                sup_arr = np.array(supply)[:, 1]
                all_drive_supply.extend(sup_arr)

            print(f"  {logname[-30:]} {mod} stator: "
                  f"mean={np.mean(np.abs(s_arr)):.1f}A  "
                  f"p95={np.percentile(np.abs(s_arr), 95):.1f}A  "
                  f"p99={np.percentile(np.abs(s_arr), 99):.1f}A  "
                  f"max={np.max(np.abs(s_arr)):.1f}A  "
                  f">{70:.0f}A: {(np.abs(s_arr) > 70).mean()*100:.1f}%")

    if all_drive_stator:
        ds = np.array(all_drive_stator)
        print(f"\n  ALL drive stator: mean={np.mean(np.abs(ds)):.1f}A  "
              f"p95={np.percentile(np.abs(ds), 95):.1f}A  "
              f"p99={np.percentile(np.abs(ds), 99):.1f}A  "
              f"max={np.max(np.abs(ds)):.1f}A")
        print(f"    >40A: {(np.abs(ds)>40).mean()*100:.2f}%  "
              f">50A: {(np.abs(ds)>50).mean()*100:.2f}%  "
              f">60A: {(np.abs(ds)>60).mean()*100:.2f}%  "
              f">70A: {(np.abs(ds)>70).mean()*100:.2f}%  "
              f">80A: {(np.abs(ds)>80).mean()*100:.2f}%")
    if all_drive_supply:
        dsup = np.array(all_drive_supply)
        print(f"\n  ALL drive supply: mean={np.mean(np.abs(dsup)):.1f}A  "
              f"p95={np.percentile(np.abs(dsup), 95):.1f}A  "
              f"p99={np.percentile(np.abs(dsup), 99):.1f}A  "
              f"max={np.max(np.abs(dsup)):.1f}A")
        print(f"    >30A: {(np.abs(dsup)>30).mean()*100:.2f}%  "
              f">40A: {(np.abs(dsup)>40).mean()*100:.2f}%  "
              f">50A: {(np.abs(dsup)>50).mean()*100:.2f}%  "
              f">60A: {(np.abs(dsup)>60).mean()*100:.2f}%")

    # ==========================================
    # TURN MOTOR CURRENTS
    # ==========================================
    print("\n" + "=" * 70)
    print("TURN MOTOR CURRENTS")
    print("  Current limits: supply=20A, stator=40A")
    print("=" * 70)

    all_turn_stator = []
    all_turn_supply = []
    for logname, data in all_data.items():
        for mod in MODULES:
            stator = data.get((mod, 'turn_stator'), [])
            supply = data.get((mod, 'turn_supply'), [])
            if len(stator) < 100:
                continue
            s_arr = np.array(stator)[:, 1]
            all_turn_stator.extend(s_arr)
            if len(supply) >= 100:
                sup_arr = np.array(supply)[:, 1]
                all_turn_supply.extend(sup_arr)

            print(f"  {logname[-30:]} {mod} stator: "
                  f"mean={np.mean(np.abs(s_arr)):.1f}A  "
                  f"p95={np.percentile(np.abs(s_arr), 95):.1f}A  "
                  f"p99={np.percentile(np.abs(s_arr), 99):.1f}A  "
                  f"max={np.max(np.abs(s_arr)):.1f}A  "
                  f">{40:.0f}A: {(np.abs(s_arr) > 40).mean()*100:.1f}%")

    if all_turn_stator:
        ts = np.array(all_turn_stator)
        print(f"\n  ALL turn stator: mean={np.mean(np.abs(ts)):.1f}A  "
              f"p95={np.percentile(np.abs(ts), 95):.1f}A  "
              f"p99={np.percentile(np.abs(ts), 99):.1f}A  "
              f"max={np.max(np.abs(ts)):.1f}A")
        print(f"    >10A: {(np.abs(ts)>10).mean()*100:.2f}%  "
              f">20A: {(np.abs(ts)>20).mean()*100:.2f}%  "
              f">30A: {(np.abs(ts)>30).mean()*100:.2f}%  "
              f">40A: {(np.abs(ts)>40).mean()*100:.2f}%")
    if all_turn_supply:
        tsup = np.array(all_turn_supply)
        print(f"\n  ALL turn supply: mean={np.mean(np.abs(tsup)):.1f}A  "
              f"p95={np.percentile(np.abs(tsup), 95):.1f}A  "
              f"p99={np.percentile(np.abs(tsup), 99):.1f}A  "
              f"max={np.max(np.abs(tsup)):.1f}A")
        print(f"    >5A: {(np.abs(tsup)>5).mean()*100:.2f}%  "
              f">10A: {(np.abs(tsup)>10).mean()*100:.2f}%  "
              f">15A: {(np.abs(tsup)>15).mean()*100:.2f}%  "
              f">20A: {(np.abs(tsup)>20).mean()*100:.2f}%")

    # ==========================================
    # POWER ANALYSIS
    # ==========================================
    print("\n" + "=" * 70)
    print("TOTAL POWER DRAW ESTIMATE (4 modules)")
    print("=" * 70)

    # Estimate per-module power from stator current * voltage
    for logname, data in list(all_data.items())[:3]:  # just a few logs
        for mod in MODULES[:1]:  # just NE
            stator = data.get((mod, 'drive_stator'), [])
            volt = data.get((mod, 'drive_volt'), [])
            t_stator = data.get((mod, 'turn_stator'), [])
            t_volt = data.get((mod, 'turn_volt'), [])
            if len(stator) < 100 or len(volt) < 100:
                continue
            s_arr = np.array(stator)
            v_arr = np.array(volt)
            t0 = max(s_arr[0, 0], v_arr[0, 0])
            t1 = min(s_arr[-1, 0], v_arr[-1, 0])
            t = np.arange(t0, t1, 0.02)
            s_interp = np.interp(t, s_arr[:, 0], s_arr[:, 1])
            v_interp = np.interp(t, v_arr[:, 0], v_arr[:, 1])
            power = np.abs(s_interp * v_interp)
            print(f"  {logname[-30:]} {mod} drive power: "
                  f"mean={np.mean(power):.0f}W  p95={np.percentile(power, 95):.0f}W  "
                  f"max={np.max(power):.0f}W")

    # ==========================================
    # SUSTAINED CURRENT ANALYSIS
    # ==========================================
    print("\n" + "=" * 70)
    print("SUSTAINED CURRENT (rolling 2s window — matches SupplyCurrentLowerTime)")
    print("=" * 70)

    for logname, data in all_data.items():
        for mod in MODULES:
            supply = data.get((mod, 'drive_supply'), [])
            if len(supply) < 200:
                continue
            sup_arr = np.array(supply)
            t = sup_arr[:, 0]
            vals = np.abs(sup_arr[:, 1])
            # Rolling 2s mean (100 samples at 50Hz)
            window = 100
            if len(vals) < window:
                continue
            cumsum = np.cumsum(vals)
            rolling = (cumsum[window:] - cumsum[:-window]) / window
            print(f"  {logname[-30:]} {mod} drive supply 2s-avg: "
                  f"mean={np.mean(rolling):.1f}A  "
                  f"p95={np.percentile(rolling, 95):.1f}A  "
                  f"max={np.max(rolling):.1f}A  "
                  f">60A: {(rolling > 60).mean()*100:.1f}%")

    # ==========================================
    # SUMMARY
    # ==========================================
    print("\n" + "=" * 70)
    print("SUMMARY & RECOMMENDATIONS")
    print("=" * 70)
    if all_drive_stator:
        ds = np.array(all_drive_stator)
        print(f"\n  DRIVE stator (limit=70A):")
        print(f"    Hitting limit: {(np.abs(ds)>70).mean()*100:.2f}% of samples")
        print(f"    Headroom to limit: p99={np.percentile(np.abs(ds),99):.0f}A vs 70A limit")
    if all_drive_supply:
        dsup = np.array(all_drive_supply)
        print(f"\n  DRIVE supply (limit=60A, 2s ramp):")
        print(f"    Hitting limit: {(np.abs(dsup)>60).mean()*100:.2f}% of samples")
    if all_turn_stator:
        ts = np.array(all_turn_stator)
        print(f"\n  TURN stator (limit=40A):")
        print(f"    Hitting limit: {(np.abs(ts)>40).mean()*100:.2f}% of samples")
        print(f"    Headroom to limit: p99={np.percentile(np.abs(ts),99):.0f}A vs 40A limit")
    if all_turn_supply:
        tsup = np.array(all_turn_supply)
        print(f"\n  TURN supply (limit=20A):")
        print(f"    Hitting limit: {(np.abs(tsup)>20).mean()*100:.2f}% of samples")

    # Kraken X60 specs
    print(f"\n  Reference: Kraken X60 specs:")
    print(f"    Stall current: 366A")
    print(f"    Free current: 2A")
    print(f"    Stall torque: 7.09 Nm")
    print(f"    Typical FRC current limits: 40-80A stator, 40-60A supply")
