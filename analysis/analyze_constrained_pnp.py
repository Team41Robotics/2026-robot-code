"""Analyze constrainedPnP output from a wpilog file."""
import struct
import math
import sys

from wpiutil.log import DataLogReader

log_path = sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-03-08_14-58-00_njwas_q49_sim.wpilog"

entry_ids = {}
reader = DataLogReader(log_path)
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entry_ids[d.entry] = (d.name, d.type)

targets = {
    "/RealOutputs//Vision/DuckyNE/constrainedPnPPose",
    "/RealOutputs//Vision/DuckyNE/nTargets",
    "/RealOutputs//Vision/DuckyNE/method",
    "/RealOutputs//Vision/DuckySE/constrainedPnPPose",
    "/RealOutputs//Vision/DuckySE/nTargets",
}
id_to_name = {eid: name for eid, (name, typ) in entry_ids.items() if name in targets}

# Field bounds (2026 Welded field approx)
FIELD_W = 17.548
FIELD_H = 8.052


def sane(x, y, z):
    return 0 <= x <= FIELD_W and 0 <= y <= FIELD_H and -2 <= z <= 3


poses_ne = []
poses_se = []
ntargets_ne = {}
methods_ne = {}

reader2 = DataLogReader(log_path)
for record in reader2:
    if record.isControl():
        continue
    eid = record.getEntry()
    if eid not in id_to_name:
        continue
    name = id_to_name[eid]
    ts = record.getTimestamp() / 1e6
    raw = record.getRaw()
    if "constrainedPnPPose" in name and len(raw) >= 56:
        vals = struct.unpack_from("<7d", raw)
        x, y, z = vals[0], vals[1], vals[2]
        qw, qx, qy, qz = vals[3], vals[4], vals[5], vals[6]
        yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
        s = sane(x, y, z)
        if "DuckyNE" in name:
            poses_ne.append((ts, x, y, z, yaw, s))
        else:
            poses_se.append((ts, x, y, z, yaw, s))
    elif "nTargets" in name and "DuckyNE" in name and len(raw) >= 8:
        ntargets_ne[ts] = struct.unpack_from("<q", raw)[0]
    elif "method" in name and "DuckyNE" in name and len(raw) > 0:
        methods_ne[ts] = raw.decode("utf-8", errors="replace")


def print_stats(poses, name):
    total = len(poses)
    sane_poses = [p for p in poses if p[5]]
    insane = [p for p in poses if not p[5]]
    print(f"=== {name} ===")
    print(f"Total constrainedPnP poses: {total}")
    print(f"Sane (on-field): {len(sane_poses)} ({100*len(sane_poses)/total:.1f}%)")
    print(f"Insane: {len(insane)} ({100*len(insane)/total:.1f}%)")
    if sane_poses:
        print(f"Sane x: {min(p[1] for p in sane_poses):.2f} to {max(p[1] for p in sane_poses):.2f}")
        print(f"Sane y: {min(p[2] for p in sane_poses):.2f} to {max(p[2] for p in sane_poses):.2f}")
        print(f"Sane yaw: {min(p[4] for p in sane_poses):.1f} to {max(p[4] for p in sane_poses):.1f} deg")
    print()

    # Show consecutive jumps among sane poses
    jumps = []
    for i in range(1, len(sane_poses)):
        prev = sane_poses[i - 1]
        cur = sane_poses[i]
        dt = cur[0] - prev[0]
        if dt > 1.0:
            continue  # big time gap, skip
        dist = math.hypot(cur[1] - prev[1], cur[2] - prev[2])
        if dist > 0.1:
            jumps.append((dist, cur[0], cur[1], cur[2], cur[4], dt))
    jumps.sort(reverse=True)
    print(f"  Pose jumps > 0.1m (among sane, dt<1s): {len(jumps)}")
    for jump in jumps[:10]:
        print(f"    t={jump[1]:.3f}s  jump={jump[0]:.3f}m  x={jump[2]:.3f}  y={jump[3]:.3f}  yaw={jump[4]:.1f}  dt={jump[5]:.3f}s")
    print()

    print("  First 10 sane poses:")
    for p in sane_poses[:10]:
        ts = p[0]
        nt = ntargets_ne.get(ts, "?") if "NE" in name else "?"
        m = methods_ne.get(ts, "?") if "NE" in name else "?"
        print(f"    t={ts:.3f}s  x={p[1]:.3f}  y={p[2]:.3f}  yaw={p[4]:.1f}  nT={nt}  method={m}")
    print()


print_stats(poses_ne, "DuckyNE")
if poses_se:
    print_stats(poses_se, "DuckySE")
