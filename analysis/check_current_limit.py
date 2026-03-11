"""Check if supply current limiting is causing drive tracking errors."""
import sys, os
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

def align_by_time(sa, sb, tol=0.015):
    if not sa or not sb: return np.array([]), np.array([]), np.array([])
    ta = np.array([x[0] for x in sa]); va = np.array([x[1] for x in sa])
    tb = np.array([x[0] for x in sb]); vb = np.array([x[1] for x in sb])
    times, ao, bo = [], [], []
    j = 0
    for i in range(len(ta)):
        while j < len(tb)-1 and tb[j+1] <= ta[i]: j += 1
        if j < len(tb) and abs(tb[j]-ta[i]) < tol:
            times.append(ta[i]); ao.append(va[i]); bo.append(vb[j])
    return np.array(times), np.array(ao), np.array(bo)

def analyze(path):
    print(f"\n{'='*80}")
    print(f"  CURRENT LIMIT ANALYSIS: {os.path.basename(path)}")
    print(f"{'='*80}")

    reader = DataLogReader(path)
    en = read_booleans(reader, {"/DriverStation/Enabled"}).get("/DriverStation/Enabled", [])
    ep = find_enabled_periods(en)

    keys = set()
    for c in ["NE","NW","SE","SW"]:
        keys.add(f"/Swerve/{c}/DriveCurrentAmps")       # stator
        keys.add(f"/Swerve/{c}/DriveBusCurrentAmps")     # supply
        keys.add(f"/Swerve/{c}/DriveVoltageVolts")
        keys.add(f"/Swerve/{c}/DriveBusVoltageVolts")
        keys.add(f"/Swerve/{c}/DriveVelMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{c}/driveErrorMetersPerSec")
        keys.add(f"/RealOutputs/Swerve/{c}/targetVelMetersPerSec")

    reader = DataLogReader(path)
    d = read_doubles(reader, keys)

    for c in ["NE","NW","SE","SW"]:
        stator = filter_enabled(d.get(f"/Swerve/{c}/DriveCurrentAmps", []), ep)
        supply = filter_enabled(d.get(f"/Swerve/{c}/DriveBusCurrentAmps", []), ep)
        err = filter_enabled(d.get(f"/RealOutputs/Swerve/{c}/driveErrorMetersPerSec", []), ep)
        vel = filter_enabled(d.get(f"/Swerve/{c}/DriveVelMetersPerSec", []), ep)
        volt = filter_enabled(d.get(f"/Swerve/{c}/DriveVoltageVolts", []), ep)
        bus_v = filter_enabled(d.get(f"/Swerve/{c}/DriveBusVoltageVolts", []), ep)
        target = filter_enabled(d.get(f"/RealOutputs/Swerve/{c}/targetVelMetersPerSec", []), ep)

        print(f"\n  [{c}]")

        if stator:
            sv = np.array([x[1] for x in stator])
            print(f"    Stator current:  mean={np.mean(np.abs(sv)):.1f}A  p95={np.percentile(np.abs(sv),95):.1f}A  max={np.max(np.abs(sv)):.1f}A")
            pct_above_90 = 100*np.sum(np.abs(sv)>90)/len(sv)
            print(f"    Above 90A stator limit: {pct_above_90:.1f}%")

        if supply:
            sv = np.array([x[1] for x in supply])
            print(f"    Supply current:  mean={np.mean(np.abs(sv)):.1f}A  p95={np.percentile(np.abs(sv),95):.1f}A  max={np.max(np.abs(sv)):.1f}A")
            pct_above_50 = 100*np.sum(np.abs(sv)>50)/len(sv)
            pct_above_60 = 100*np.sum(np.abs(sv)>60)/len(sv)
            print(f"    Above 50A: {pct_above_50:.1f}%   Above 60A supply limit: {pct_above_60:.1f}%")

        # Correlate: when supply current is near/at limit, what's the error?
        if supply and err:
            t, sup, er = align_by_time(supply, err)
            if len(t) > 50:
                at_limit = np.abs(sup) > 55  # near the 60A limit
                not_limit = np.abs(sup) <= 55
                if np.sum(at_limit) > 10 and np.sum(not_limit) > 10:
                    err_at_limit = np.mean(np.abs(er[at_limit]))
                    err_not_limit = np.mean(np.abs(er[not_limit]))
                    print(f"    Error when supply > 55A: {err_at_limit:.3f} m/s")
                    print(f"    Error when supply < 55A: {err_not_limit:.3f} m/s")
                    if err_at_limit > err_not_limit * 1.5:
                        print(f"    ** CURRENT LIMITING IS CAUSING TRACKING ERROR **")

        # Also check: what % of time at high target velocity is current-limited?
        if supply and target:
            t, sup, tgt = align_by_time(supply, target)
            if len(t) > 50:
                high_tgt = np.abs(tgt) > 2.0
                if np.sum(high_tgt) > 10:
                    pct_limited_at_high = 100*np.sum(np.abs(sup[high_tgt]) > 55) / np.sum(high_tgt)
                    print(f"    Current-limited when |target| > 2 m/s: {pct_limited_at_high:.1f}%")

        # Power analysis: V * I at the motor
        if volt and stator:
            t, v, i = align_by_time(volt, stator)
            if len(t) > 50:
                power = v * i
                print(f"    Motor power: mean={np.mean(np.abs(power)):.0f}W  max={np.max(np.abs(power)):.0f}W")

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
