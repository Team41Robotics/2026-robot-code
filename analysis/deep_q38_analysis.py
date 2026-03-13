"""Deep analysis of q38 vision data: constrainedPnP vs pnpDistTrig vs pose estimator."""
import struct
import math
import sys
import collections
import bisect

from wpiutil.log import DataLogReader

log_path = sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-03-07_23-51-56_njwas_q38_sim.wpilog"

# --- Pass 1: collect entry IDs ---
entry_ids = {}
reader = DataLogReader(log_path)
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entry_ids[d.entry] = (d.name, d.type)

# Print all log entries for drive/vision under ReplayOutputs
print("=== ReplayOutputs entries (Drive + Vision + Odom) ===")
for eid, (name, typ) in sorted(entry_ids.items(), key=lambda x: x[1][0]):
    if name.startswith("/ReplayOutputs/") and ("Vision" in name or "Drive" in name or "Odom" in name or "drive" in name.lower()):
        print(f"  {name}  [{typ}]")
# Also show RealOutputs drive/odom
print("\n=== RealOutputs entries (Drive + Odom) ===")
for eid, (name, typ) in sorted(entry_ids.items(), key=lambda x: x[1][0]):
    if name.startswith("/RealOutputs/") and ("Drive" in name or "Odom" in name or "drive" in name.lower() or "pose" in name.lower()):
        print(f"  {name}  [{typ}]")
print()

# --- Gather targets ---
cam_names = ["DuckyNE", "DuckySE"]
wanted_suffixes = [
    "constrainedPnPPose", "coprocPnPPose", "pnpDistTrigPose",
    "estimatedPose", "nTargets", "method", "covXY", "covTheta", "avgDist",
]

# Use ReplayOutputs for vision (current code)
targets = set()
for cam in cam_names:
    for s in wanted_suffixes:
        targets.add(f"/ReplayOutputs//Vision/{cam}/{s}")

# Find drive pose - try common paths
drive_pose_candidates = []
for eid, (name, typ) in entry_ids.items():
    if "pose" in name.lower() and ("Drive" in name or "Odom" in name or "drive" in name.lower()):
        drive_pose_candidates.append((name, typ))
print(f"Drive pose candidates: {drive_pose_candidates}")

# Also grab the real-time drive pose from RealOutputs (this was the pose used during the match)
# and the IMU yaw
extra_keys = set()
for eid, (name, typ) in entry_ids.items():
    if "pose" in name.lower() and "RealOutputs" in name:
        extra_keys.add(name)
    if "imuYaw" in name:
        extra_keys.add(name)
targets.update(extra_keys)

id_to_name = {eid: name for eid, (name, typ) in entry_ids.items() if name in targets}

# --- Pass 2: extract data ---
FIELD_W = 17.548
FIELD_H = 8.052

def decode_pose3d(raw):
    if len(raw) < 56:
        return None
    vals = struct.unpack_from("<7d", raw)
    x, y, z = vals[0], vals[1], vals[2]
    qw, qx, qy, qz = vals[3], vals[4], vals[5], vals[6]
    yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return (x, y, z, yaw)

def decode_pose2d(raw):
    if len(raw) < 24:
        return None
    vals = struct.unpack_from("<3d", raw)
    return (vals[0], vals[1], math.degrees(vals[2]))

def sane(x, y, z):
    return 0 <= x <= FIELD_W and 0 <= y <= FIELD_H and -2 <= z <= 3

# Per-camera time series
data = {cam: {s: {} for s in wanted_suffixes} for cam in cam_names}
drive_poses = {}  # name -> {ts -> pose}
imu_yaw = {}

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

    # Drive/odom poses
    if "pose" in name.lower() and "Vision" not in name:
        p = decode_pose2d(raw)
        if p:
            if name not in drive_poses:
                drive_poses[name] = {}
            drive_poses[name][ts] = p
        continue

    if "imuYaw" in name and len(raw) >= 8:
        imu_yaw[ts] = math.degrees(struct.unpack_from("<d", raw)[0])
        continue

    # Vision data - match to camera
    for cam in cam_names:
        prefix = f"/ReplayOutputs//Vision/{cam}/"
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix):]
        if suffix not in wanted_suffixes:
            break

        if suffix.endswith("Pose") and suffix != "estimatedPose":
            p = decode_pose3d(raw)
            if p:
                data[cam][suffix][ts] = p
        elif suffix == "estimatedPose":
            p = decode_pose2d(raw)
            if p:
                data[cam][suffix][ts] = p
        elif suffix == "nTargets" and len(raw) >= 8:
            data[cam][suffix][ts] = struct.unpack_from("<q", raw)[0]
        elif suffix == "method" and len(raw) > 0:
            data[cam][suffix][ts] = raw.decode("utf-8", errors="replace")
        elif suffix in ("covXY", "covTheta", "avgDist") and len(raw) >= 8:
            data[cam][suffix][ts] = struct.unpack_from("<d", raw)[0]
        break

# Pick the best drive pose reference
print(f"\nDrive pose sources found:")
for name, poses in drive_poses.items():
    print(f"  {name}: {len(poses)} samples")

# Use the one with most samples as reference
ref_name = max(drive_poses, key=lambda k: len(drive_poses[k])) if drive_poses else None
if ref_name:
    ref_poses = drive_poses[ref_name]
    ref_ts_sorted = sorted(ref_poses.keys())
    print(f"Using {ref_name} as reference ({len(ref_poses)} samples)")
else:
    ref_poses = {}
    ref_ts_sorted = []
    print("WARNING: No drive pose reference found!")

print(f"IMU yaw samples: {len(imu_yaw)}")
print()

def find_nearest_ref(t):
    if not ref_ts_sorted:
        return None
    idx = bisect.bisect_left(ref_ts_sorted, t)
    if idx == 0:
        return ref_poses[ref_ts_sorted[0]]
    if idx >= len(ref_ts_sorted):
        return ref_poses[ref_ts_sorted[-1]]
    t0, t1 = ref_ts_sorted[idx-1], ref_ts_sorted[idx]
    best_t = t0 if (t - t0) < (t1 - t) else t1
    if abs(best_t - t) > 0.1:
        return None
    return ref_poses[best_t]

for cam in cam_names:
    print(f"{'='*60}")
    print(f"  Camera: {cam}")
    print(f"{'='*60}")

    cpnp = data[cam]["constrainedPnPPose"]
    trigpnp = data[cam]["pnpDistTrigPose"]
    coproc = data[cam]["coprocPnPPose"]
    methods = data[cam]["method"]
    ntargets = data[cam]["nTargets"]
    estimated = data[cam]["estimatedPose"]
    avg_dists = data[cam]["avgDist"]

    print(f"  constrainedPnP samples: {len(cpnp)}")
    print(f"  pnpDistTrig samples:    {len(trigpnp)}")
    print(f"  coprocPnP samples:      {len(coproc)}")
    print(f"  method samples:         {len(methods)}")
    print(f"  nTargets samples:       {len(ntargets)}")
    print(f"  estimatedPose samples:  {len(estimated)}")
    print()

    # Method distribution
    method_counts = collections.Counter(methods.values())
    print(f"  Method distribution:")
    for m, c in method_counts.most_common():
        print(f"    {m}: {c}")
    print()

    # nTargets distribution when constrainedPnP has a result
    cpnp_times = sorted(cpnp.keys())
    ntargets_sorted = sorted(ntargets.keys())

    def find_nearest_ntargets(t):
        if not ntargets_sorted:
            return None
        idx = bisect.bisect_left(ntargets_sorted, t)
        candidates = []
        if idx > 0:
            candidates.append(ntargets_sorted[idx-1])
        if idx < len(ntargets_sorted):
            candidates.append(ntargets_sorted[idx])
        best = min(candidates, key=lambda x: abs(x - t))
        if abs(best - t) < 0.05:
            return ntargets[best]
        return None

    def find_nearest_avgdist(t):
        if not avg_dists:
            return None
        ad_sorted = sorted(avg_dists.keys())
        idx = bisect.bisect_left(ad_sorted, t)
        candidates = []
        if idx > 0:
            candidates.append(ad_sorted[idx-1])
        if idx < len(ad_sorted):
            candidates.append(ad_sorted[idx])
        best = min(candidates, key=lambda x: abs(x - t))
        if abs(best - t) < 0.05:
            return avg_dists[best]
        return None

    nt_counter = collections.Counter()
    for t in cpnp_times:
        nt = find_nearest_ntargets(t)
        if nt is not None:
            nt_counter[nt] += 1
    print(f"  nTargets distribution when constrainedPnP has a result:")
    for nt, c in sorted(nt_counter.items()):
        print(f"    {nt} tags: {c} times")
    print()

    # Sanity rate
    sane_count = sum(1 for t in cpnp_times if sane(cpnp[t][0], cpnp[t][1], cpnp[t][2]))
    print(f"  constrainedPnP sane: {sane_count}/{len(cpnp_times)} ({100*sane_count/max(1,len(cpnp_times)):.1f}%)")

    # Error vs reference pose, broken down by nTargets
    errors_by_nt = collections.defaultdict(list)
    yaw_errors_by_nt = collections.defaultdict(list)
    for t in cpnp_times:
        cp = cpnp[t]
        rp = find_nearest_ref(t)
        if rp is None or not sane(cp[0], cp[1], cp[2]):
            continue
        xy_err = math.hypot(cp[0] - rp[0], cp[1] - rp[1])
        yaw_err = abs((cp[3] - rp[2] + 180) % 360 - 180)
        nt = find_nearest_ntargets(t) or 0
        errors_by_nt[nt].append(xy_err)
        yaw_errors_by_nt[nt].append(yaw_err)

    print(f"\n  constrainedPnP XY error vs reference pose (by nTargets):")
    for nt in sorted(errors_by_nt.keys()):
        errs = sorted(errors_by_nt[nt])
        yerrs = sorted(yaw_errors_by_nt[nt])
        n = len(errs)
        print(f"    {nt} tags: n={n}  xy_p50={errs[n//2]:.3f}m  xy_p90={errs[int(n*0.9)]:.3f}m  xy_p99={errs[int(n*0.99)]:.3f}m  yaw_p50={yerrs[n//2]:.1f}°  yaw_p90={yerrs[int(n*0.9)]:.1f}°")

    # Same for pnpDistTrig
    trigpnp_times = sorted(trigpnp.keys())
    trig_errors = []
    trig_yaw_errors = []
    for t in trigpnp_times:
        tp = trigpnp[t]
        rp = find_nearest_ref(t)
        if rp is None or not sane(tp[0], tp[1], tp[2]):
            continue
        trig_errors.append(math.hypot(tp[0] - rp[0], tp[1] - rp[1]))
        trig_yaw_errors.append(abs((tp[3] - rp[2] + 180) % 360 - 180))

    if trig_errors:
        te = sorted(trig_errors)
        ty = sorted(trig_yaw_errors)
        n = len(te)
        print(f"\n  pnpDistTrig XY error vs reference: n={n}")
        print(f"    xy_p50={te[n//2]:.3f}m  xy_p90={te[int(n*0.9)]:.3f}m  xy_p99={te[int(n*0.99)]:.3f}m")
        print(f"    yaw_p50={ty[n//2]:.1f}°  yaw_p90={ty[int(n*0.9)]:.1f}°")

    # Frame-to-frame jumps
    sane_cpnp = [(t, cpnp[t]) for t in cpnp_times if sane(cpnp[t][0], cpnp[t][1], cpnp[t][2])]
    jumps = []
    for i in range(1, len(sane_cpnp)):
        t0, p0 = sane_cpnp[i-1]
        t1, p1 = sane_cpnp[i]
        dt = t1 - t0
        if dt > 0.5:
            continue
        jumps.append(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    if jumps:
        js = sorted(jumps)
        n = len(js)
        print(f"\n  constrainedPnP frame-to-frame jumps (sane, dt<0.5s): n={n}")
        print(f"    p50={js[n//2]:.3f}m  p75={js[int(n*0.75)]:.3f}m  p90={js[int(n*0.9)]:.3f}m  p99={js[int(n*0.99)]:.3f}m  max={js[-1]:.3f}m")

    # Head-to-head: constrainedPnP vs pnpDistTrig at matching timestamps
    trigpnp_ts_sorted = sorted(trigpnp.keys())
    common_count = 0
    cpnp_better = 0
    trig_better = 0
    diffs = []
    for t in cpnp_times:
        idx = bisect.bisect_left(trigpnp_ts_sorted, t)
        candidates = []
        if idx > 0: candidates.append(trigpnp_ts_sorted[idx-1])
        if idx < len(trigpnp_ts_sorted): candidates.append(trigpnp_ts_sorted[idx])
        if not candidates: continue
        best_tt = min(candidates, key=lambda x: abs(x - t))
        if abs(best_tt - t) > 0.01: continue

        cp = cpnp[t]
        tp = trigpnp[best_tt]
        rp = find_nearest_ref(t)
        if rp is None or not (sane(cp[0], cp[1], cp[2]) and sane(tp[0], tp[1], tp[2])):
            continue
        cp_err = math.hypot(cp[0] - rp[0], cp[1] - rp[1])
        tp_err = math.hypot(tp[0] - rp[0], tp[1] - rp[1])
        common_count += 1
        if cp_err < tp_err:
            cpnp_better += 1
        else:
            trig_better += 1
        diffs.append(cp_err - tp_err)

    if diffs:
        ds = sorted(diffs)
        n = len(ds)
        print(f"\n  Head-to-head (constrainedPnP_err - pnpDistTrig_err):")
        print(f"    n={common_count}  cpnp_wins={cpnp_better}  trig_wins={trig_better}")
        print(f"    avg_diff={sum(diffs)/n:.3f}m  (negative = constrainedPnP better)")
        print(f"    p10={ds[int(n*0.1)]:.3f}m  p50={ds[n//2]:.3f}m  p90={ds[int(n*0.9)]:.3f}m")

    # Avg tag distance when bad vs good
    bad_dists_list = []
    good_dists_list = []
    for t in cpnp_times:
        cp = cpnp[t]
        rp = find_nearest_ref(t)
        if rp is None or not sane(cp[0], cp[1], cp[2]):
            continue
        err = math.hypot(cp[0] - rp[0], cp[1] - rp[1])
        ad = find_nearest_avgdist(t)
        if ad is None:
            continue
        if err > 1.0:
            bad_dists_list.append((ad, err, t))
        else:
            good_dists_list.append((ad, err, t))

    if bad_dists_list:
        print(f"\n  Avg tag dist when constrainedPnP error > 1m: {sum(d[0] for d in bad_dists_list)/len(bad_dists_list):.2f}m  (n={len(bad_dists_list)}, avg_err={sum(d[1] for d in bad_dists_list)/len(bad_dists_list):.2f}m)")
    if good_dists_list:
        print(f"  Avg tag dist when constrainedPnP error < 1m: {sum(d[0] for d in good_dists_list)/len(good_dists_list):.2f}m  (n={len(good_dists_list)}, avg_err={sum(d[1] for d in good_dists_list)/len(good_dists_list):.2f}m)")

    # Heading analysis
    print(f"\n  Heading analysis (constrainedPnP yaw vs ref yaw):")
    yaw_small = []
    yaw_large = []
    for t in cpnp_times:
        cp = cpnp[t]
        rp = find_nearest_ref(t)
        if rp is None or not sane(cp[0], cp[1], cp[2]):
            continue
        xy_err = math.hypot(cp[0] - rp[0], cp[1] - rp[1])
        yaw_diff = abs((cp[3] - rp[2] + 180) % 360 - 180)
        if xy_err < 0.5:
            yaw_small.append(yaw_diff)
        elif xy_err > 1.0:
            yaw_large.append(yaw_diff)
    if yaw_small:
        s = sorted(yaw_small); n = len(s)
        print(f"    When xy_err < 0.5m: n={n}  yaw_p50={s[n//2]:.1f}°  yaw_p90={s[int(n*0.9)]:.1f}°")
    if yaw_large:
        s = sorted(yaw_large); n = len(s)
        print(f"    When xy_err > 1.0m: n={n}  yaw_p50={s[n//2]:.1f}°  yaw_p90={s[int(n*0.9)]:.1f}°")

    # 1-tag vs multi-tag constrainedPnP: are 1-tag results the problem?
    print(f"\n  constrainedPnP sane rate by nTargets:")
    for nt_val in sorted(nt_counter.keys()):
        total_nt = 0
        sane_nt = 0
        for t in cpnp_times:
            nt = find_nearest_ntargets(t)
            if nt != nt_val:
                continue
            total_nt += 1
            if sane(cpnp[t][0], cpnp[t][1], cpnp[t][2]):
                sane_nt += 1
        print(f"    {nt_val} tags: {sane_nt}/{total_nt} sane ({100*sane_nt/max(1,total_nt):.1f}%)")

    print()
