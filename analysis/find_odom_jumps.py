"""Find large odom pose jumps in a specific log file."""
import sys, os, math, struct
from collections import defaultdict
from wpiutil.log import DataLogReader
import numpy as np

def read_struct_pose2d(reader, keys):
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
                    raw = record.getRaw()
                    if len(raw) >= 24:
                        x, y, cos_a, sin_a = struct.unpack_from('<dddd', raw, 0)
                        angle = math.atan2(sin_a, cos_a)
                        data[entries[eid]].append((record.getTimestamp() / 1e6, x, y, angle))
                except Exception:
                    pass
    return data

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

def analyze(path):
    print(f"Analyzing: {os.path.basename(path)}")
    print(f"{'='*80}")

    # Read odom pose from doubles
    reader = DataLogReader(path)
    dbl = read_doubles(reader, {
        "/RealOutputs//Odom/xMeters",
        "/RealOutputs//Odom/yMeters",
        "/RealOutputs//Odom/rotRadians",
    })
    ox = dbl.get("/RealOutputs//Odom/xMeters", [])
    oy = dbl.get("/RealOutputs//Odom/yMeters", [])
    orot = dbl.get("/RealOutputs//Odom/rotRadians", [])

    if not ox or not oy:
        print("No odom data found")
        return

    # Align by index (they should be logged at the same time)
    n = min(len(ox), len(oy), len(orot))
    odom = [(ox[i][0], ox[i][1], oy[i][1], orot[i][1] if i < len(orot) else 0) for i in range(n)]

    # Read enabled
    reader = DataLogReader(path)
    en = read_booleans(reader, {"/DriverStation/Enabled"}).get("/DriverStation/Enabled", [])

    def is_enabled(t):
        last = False
        for et, ev in en:
            if et > t: break
            last = ev
        return last

    # Find all jumps > threshold
    print(f"\nTotal odom samples: {len(odom)}")
    print(f"Time range: {odom[0][0]:.1f}s - {odom[-1][0]:.1f}s")

    for threshold in [0.5, 0.2, 0.1, 0.05]:
        jumps = []
        for i in range(1, len(odom)):
            t0, x0, y0, a0 = odom[i-1]
            t1, x1, y1, a1 = odom[i]
            if not is_enabled(t1):
                continue
            dt = t1 - t0
            if dt <= 0 or dt > 1.0:
                continue
            dist = math.hypot(x1 - x0, y1 - y0)
            da = abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
            if dist > threshold:
                jumps.append((t1, dist, da, dt, x0, y0, x1, y1))

        print(f"\nJumps > {threshold}m: {len(jumps)}")
        if jumps and threshold >= 0.1:
            print(f"  {'time':>8s}  {'dist':>6s}  {'dtheta':>7s}  {'dt':>6s}  {'from':>16s}  {'to':>16s}")
            for t, d, da, dt, x0, y0, x1, y1 in sorted(jumps, key=lambda x: -x[1])[:20]:
                print(f"  {t:8.2f}s  {d:5.3f}m  {math.degrees(da):6.1f}deg  {dt*1000:5.1f}ms  ({x0:6.2f},{y0:6.2f})  ({x1:6.2f},{y1:6.2f})")

    # Also show odom total drift: distance between start and end pose
    t0, x0, y0, a0 = odom[0]
    tf, xf, yf, af = odom[-1]
    print(f"\nStart pose: ({x0:.2f}, {y0:.2f}) @ {math.degrees(a0):.1f}deg")
    print(f"End pose:   ({xf:.2f}, {yf:.2f}) @ {math.degrees(af):.1f}deg")
    print(f"Total displacement: {math.hypot(xf-x0, yf-y0):.2f}m")

    # Show odom XY over time to see drift pattern
    print(f"\nOdom position every 10s (enabled only):")
    print(f"  {'time':>8s}  {'x':>7s}  {'y':>7s}  {'heading':>8s}")
    last_print = -999
    for t, x, y, a in odom:
        if not is_enabled(t):
            continue
        if t - last_print >= 10:
            print(f"  {t:8.2f}s  {x:7.2f}m  {y:7.2f}m  {math.degrees(a):7.1f}deg")
            last_print = t

def main():
    p = sys.argv[1] if len(sys.argv) > 1 else "../logs/akit_26-03-07_23-51-56_njwas_q38.wpilog"
    if os.path.isfile(p):
        analyze(p)
    else:
        print(f"File not found: {p}")

if __name__ == "__main__":
    main()
