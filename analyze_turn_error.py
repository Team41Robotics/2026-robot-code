"""Analyze turn error signals from day-1 match logs."""
import numpy as np
from wpiutil.log import DataLogReader

LOG_DIR = "c:/Users/Robotics41/Desktop/2026-robot-code-new/logs/"
LOG_FILES = [
    "akit_26-03-07_14-54-34_njwas_p4.wpilog",
    "akit_26-03-07_17-35-44_njwas_q9.wpilog",
    "akit_26-03-07_19-33-25_njwas_q15.wpilog",
]

MODULES = ["NE", "NW", "SE"]

# Signal name patterns
def error_name(mod):
    return f"/RealOutputs/Swerve/{mod}/turnErrorRadians"

def vel_name(mod):
    return f"/Swerve/{mod}/TurnVelRadiansPerSec"

def voltage_name(mod):
    return f"/Swerve/{mod}/TurnVoltageVolts"


def parse_log(path):
    """Return dict of {signal_name: [(timestamp_us, value), ...]}."""
    reader = DataLogReader(path)
    # First pass: build entry_id -> name map
    entry_names = {}
    # Collect signals we care about
    wanted = set()
    for mod in MODULES:
        wanted.add(error_name(mod))
        wanted.add(vel_name(mod))
        wanted.add(voltage_name(mod))

    data = {name: ([], []) for name in wanted}

    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in wanted:
                entry_names[sd.entry] = sd.name
            continue
        if record.isControl():
            continue
        eid = record.getEntry()
        if eid in entry_names:
            name = entry_names[eid]
            try:
                val = record.getDouble()
                data[name][0].append(record.getTimestamp())
                data[name][1].append(val)
            except Exception:
                pass
    # Convert to numpy
    return {k: (np.array(v[0], dtype=np.float64), np.array(v[1], dtype=np.float64)) for k, v in data.items()}


def merge_data(all_data):
    """Merge list of data dicts, concatenating arrays."""
    merged = {}
    for d in all_data:
        for k, (ts, vals) in d.items():
            if k not in merged:
                merged[k] = (ts, vals)
            else:
                merged[k] = (np.concatenate([merged[k][0], ts]),
                             np.concatenate([merged[k][1], vals]))
    return merged


def nearest_values(ts_target, ts_source, vals_source):
    """For each timestamp in ts_target, find nearest value in ts_source/vals_source."""
    idx = np.searchsorted(ts_source, ts_target)
    idx = np.clip(idx, 1, len(ts_source) - 1)
    # Pick whichever neighbor is closer
    left = np.abs(ts_target - ts_source[idx - 1])
    right = np.abs(ts_target - ts_source[idx])
    best = np.where(left < right, idx - 1, idx)
    return vals_source[best]


print("Parsing logs...")
all_data = []
for f in LOG_FILES:
    print(f"  {f}")
    all_data.append(parse_log(LOG_DIR + f))

data = merge_data(all_data)

print(f"\n{'='*70}")
print("TURN ERROR ANALYSIS - Day 1 (3 logs, modules NE/NW/SE)")
print(f"{'='*70}")

# Per-module stats
for mod in MODULES:
    ename = error_name(mod)
    ts_err, err = data[ename]
    if len(err) == 0:
        print(f"\n{mod}: no data")
        continue
    abs_err = np.abs(err)

    print(f"\n--- Module {mod} ({len(err)} samples) ---")
    print(f"  Mean error:        {np.mean(err):+.5f} rad ({np.degrees(np.mean(err)):+.3f} deg)")
    print(f"  Std dev:           {np.std(err):.5f} rad ({np.degrees(np.std(err)):.3f} deg)")
    print(f"  Max |error|:       {np.max(abs_err):.5f} rad ({np.degrees(np.max(abs_err)):.3f} deg)")
    print(f"  95th pctl |error|: {np.percentile(abs_err, 95):.5f} rad ({np.degrees(np.percentile(abs_err, 95)):.3f} deg)")
    print(f"  99th pctl |error|: {np.percentile(abs_err, 99):.5f} rad ({np.degrees(np.percentile(abs_err, 99)):.3f} deg)")

    # Threshold counts
    for thresh_rad, thresh_deg in [(0.05, 3), (0.1, 6), (0.3, 17)]:
        count = np.sum(abs_err > thresh_rad)
        pct = 100.0 * count / len(err)
        print(f"  |error| > {thresh_rad:.2f} rad ({thresh_deg:2d} deg): {count:6d} samples ({pct:.2f}%)")

# Combined stats
print(f"\n{'='*70}")
print("COMBINED (all 3 modules)")
print(f"{'='*70}")
all_err = np.concatenate([data[error_name(mod)][1] for mod in MODULES])
abs_all = np.abs(all_err)
print(f"  Total samples:     {len(all_err)}")
print(f"  Mean error:        {np.mean(all_err):+.5f} rad ({np.degrees(np.mean(all_err)):+.3f} deg)")
print(f"  Std dev:           {np.std(all_err):.5f} rad ({np.degrees(np.std(all_err)):.3f} deg)")
print(f"  Max |error|:       {np.max(abs_all):.5f} rad ({np.degrees(np.max(abs_all)):.3f} deg)")
print(f"  95th pctl |error|: {np.percentile(abs_all, 95):.5f} rad ({np.degrees(np.percentile(abs_all, 95)):.3f} deg)")
print(f"  99th pctl |error|: {np.percentile(abs_all, 99):.5f} rad ({np.degrees(np.percentile(abs_all, 99)):.3f} deg)")
for thresh_rad, thresh_deg in [(0.05, 3), (0.1, 6), (0.3, 17)]:
    count = np.sum(abs_all > thresh_rad)
    pct = 100.0 * count / len(all_err)
    print(f"  |error| > {thresh_rad:.2f} rad ({thresh_deg:2d} deg): {count:6d} samples ({pct:.2f}%)")

# Correlation analysis: velocity and voltage at moments of large error
print(f"\n{'='*70}")
print("CORRELATION: LARGE ERRORS vs VELOCITY / VOLTAGE")
print(f"{'='*70}")

for mod in MODULES:
    ename = error_name(mod)
    vname = vel_name(mod)
    voltname = voltage_name(mod)

    ts_err, err = data[ename]
    ts_vel, vel = data[vname]
    ts_volt, volt = data[voltname]

    if len(err) == 0 or len(vel) == 0 or len(volt) == 0:
        print(f"\n{mod}: insufficient data for correlation")
        continue

    abs_err = np.abs(err)
    # Get velocity and voltage at error timestamps
    vel_at_err = nearest_values(ts_err, ts_vel, vel)
    volt_at_err = nearest_values(ts_err, ts_volt, volt)
    abs_vel = np.abs(vel_at_err)
    abs_volt = np.abs(volt_at_err)

    print(f"\n--- Module {mod} ---")

    # Overall correlation
    corr_vel = np.corrcoef(abs_err, abs_vel)[0, 1]
    corr_volt = np.corrcoef(abs_err, abs_volt)[0, 1]
    print(f"  Correlation(|error|, |vel|):     {corr_vel:+.4f}")
    print(f"  Correlation(|error|, |voltage|): {corr_volt:+.4f}")

    # Compare velocity stats for large vs small errors
    large_mask = abs_err > 0.05
    small_mask = abs_err <= 0.05
    if np.any(large_mask) and np.any(small_mask):
        print(f"  When |error| > 0.05 rad:")
        print(f"    Mean |vel|:     {np.mean(abs_vel[large_mask]):.3f} rad/s")
        print(f"    Mean |voltage|: {np.mean(abs_volt[large_mask]):.3f} V")
        print(f"  When |error| <= 0.05 rad:")
        print(f"    Mean |vel|:     {np.mean(abs_vel[small_mask]):.3f} rad/s")
        print(f"    Mean |voltage|: {np.mean(abs_volt[small_mask]):.3f} V")

    # Velocity percentiles when large errors happen
    for thresh in [0.1, 0.3]:
        mask = abs_err > thresh
        if np.sum(mask) > 10:
            v = abs_vel[mask]
            print(f"  When |error| > {thresh} rad: median |vel| = {np.median(v):.2f}, "
                  f"90th pctl |vel| = {np.percentile(v, 90):.2f} rad/s "
                  f"({np.sum(mask)} samples)")

# Temporal pattern: do large errors cluster at certain times?
print(f"\n{'='*70}")
print("TEMPORAL PATTERN: WHEN DO LARGE ERRORS HAPPEN?")
print(f"{'='*70}")

for mod in MODULES:
    ename = error_name(mod)
    ts_err, err = data[ename]
    abs_err = np.abs(err)

    # Normalize timestamps to seconds from start of each log
    # Just look at relative timing within each match
    large_mask = abs_err > 0.1
    if np.sum(large_mask) < 5:
        print(f"\n{mod}: too few large errors to analyze temporal pattern")
        continue

    ts_large = ts_err[large_mask]
    # Compute inter-arrival times
    dt = np.diff(ts_large) / 1e6  # microseconds to seconds
    print(f"\n--- Module {mod} ---")
    print(f"  Large errors (>0.1 rad): {np.sum(large_mask)} occurrences")

    # Check burstiness: are large errors clustered?
    bursts = np.sum(dt < 0.1)  # within 100ms of each other
    print(f"  Consecutive large errors (<100ms apart): {bursts} ({100*bursts/max(1,len(dt)):.1f}%)")
    bursts_500 = np.sum(dt < 0.5)
    print(f"  Consecutive large errors (<500ms apart): {bursts_500} ({100*bursts_500/max(1,len(dt)):.1f}%)")

    # Velocity at large error moments
    ts_vel, vel = data[vel_name(mod)]
    vel_at_large = nearest_values(ts_large, ts_vel, vel)
    print(f"  Velocity at large-error moments:")
    print(f"    Mean:   {np.mean(vel_at_large):+.3f} rad/s")
    print(f"    Median: {np.median(vel_at_large):+.3f} rad/s")
    print(f"    Std:    {np.std(vel_at_large):.3f} rad/s")
    # Sign analysis: are errors during acceleration (vel changing sign)?
    print(f"    Fraction with |vel| > 5 rad/s: {100*np.mean(np.abs(vel_at_large) > 5):.1f}%")
    print(f"    Fraction with |vel| > 10 rad/s: {100*np.mean(np.abs(vel_at_large) > 10):.1f}%")

print("\nDone.")
