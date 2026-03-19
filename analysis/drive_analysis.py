"""Drive motor response timescale analysis from day 1 match data (NE, NW, SE).

Plant model: tau*dv/dt = Kv*V(t-delay) - v - kS*sign(v)
FRC convention: V = kS*sign(v) + kV*v + kA*dv/dt
  where kV = 1/Kv, kA = tau/Kv, kS_frc = kS/Kv

Current drive FF constants (for comparison):
  kS = 0.093052 V
  kV = 1.8968 V/(m/s)  =>  Kv = 0.527 (m/s)/V
  kA = 0.015 V/(m/s^2)
"""
from wpiutil.log import DataLogReader
import numpy as np
from scipy.optimize import differential_evolution
import glob as globmod
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

LOG_DIR = 'c:/Users/Robotics41/Desktop/2026-robot-code-new/logs'
MODULES = ['NE', 'NW', 'SE']  # skip SW per user request
WINDOW = 500   # 10s @ 50Hz
STRIDE = 500   # no overlap
dt_u = 0.020

# ---- Pre-filter thresholds ----
MIN_ACTIVE_FRAC = 0.4   # fraction of samples with |V| > 0.2
MIN_VEL_STD = 0.3       # m/s — reject windows with barely any motion
MIN_VOLT_STD = 0.3      # V — reject windows with nearly constant voltage

# ---- Post-filter thresholds (for "good" fits) ----
MAX_TAU = 0.15           # 150ms — drive motors have more inertia than turn
MIN_R2 = 0.5
MAX_TAU_BOUNDARY = 0.48  # reject fits near the upper bound (0.5)

# ---- PLANT MODEL ----
def simulate_plant(params, t_arr, V_arr, v0):
    tau_m, Kv, delay, kS = params
    n = len(t_arr)
    v = np.empty(n); v[0] = v0
    delay_steps = max(0, int(round(delay / dt_u)))
    for i in range(n-1):
        V_d = V_arr[max(0, i - delay_steps)]
        dvdt = (Kv * V_d - v[i] - kS * np.sign(v[i])) / tau_m
        v[i+1] = v[i] + dt_u * dvdt
        if abs(v[i+1]) > 1e4:
            v[i+1:] = 1e4
            break
    return v

def cost(params, t_arr, V_arr, v_meas):
    tau_m, Kv, delay, kS = params
    if tau_m < 0.005 or Kv < 0.05:
        return 1e9
    v_sim = simulate_plant(params, t_arr, V_arr, v_meas[0])
    return np.mean((v_sim - v_meas)**2)

def fit_one_window(args):
    """Fit a single window -- runs in worker process."""
    t_w, V_w, v_w, mod, logname = args
    res = differential_evolution(
        cost, args=(t_w, V_w, v_w),
        bounds=[(0.005, 0.5), (0.05, 3.0), (0.0, 0.08), (0.0, 3.0)],
        seed=42, maxiter=80, tol=1e-5, workers=1, popsize=10
    )
    tau_f, Kv_f, del_f, kS_f = res.x
    rmse = np.sqrt(res.fun)
    v_var = v_w.var()
    r2 = 1 - res.fun / v_var if v_var > 0 else -999
    return {
        't0': t_w[0], 'tau': tau_f, 'Kv': Kv_f, 'delay': del_f,
        'kS': kS_f, 'rmse': rmse, 'v_std': v_w.std(), 'V_std': V_w.std(),
        'r2': r2, 'mod': mod, 'log': logname
    }

def load_segments(log_paths):
    all_segments = []
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
                    if name == f'/Swerve/{mod}/DriveVelMetersPerSec':
                        data.setdefault((mod, 'vel'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))
                    elif name == f'/Swerve/{mod}/DriveVoltageVolts':
                        data.setdefault((mod, 'volt'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))

        for mod in MODULES:
            vel_data = data.get((mod, 'vel'), [])
            volt_data = data.get((mod, 'volt'), [])
            if len(vel_data) < 100 or len(volt_data) < 100:
                continue
            vel_arr = np.array(vel_data)
            volt_arr = np.array(volt_data)
            t_vel = vel_arr[:, 0]; vel = vel_arr[:, 1]
            t_volt = volt_arr[:, 0]; volt = volt_arr[:, 1]
            t0 = max(t_vel[0], t_volt[0])
            t1 = min(t_vel[-1], t_volt[-1])
            if t1 - t0 < 5.0:
                continue
            t_unif = np.arange(t0, t1, dt_u)
            v_unif = np.interp(t_unif, t_vel, vel)
            V_unif = np.interp(t_unif, t_volt, volt)
            all_segments.append((t_unif, v_unif, V_unif, mod, os.path.basename(log_path)))
    return all_segments

def is_good_fit(r):
    """Post-filter: reject degenerate fits."""
    return (r['tau'] < MAX_TAU
            and r['tau'] < MAX_TAU_BOUNDARY
            and r['r2'] > MIN_R2
            and r['Kv'] < 2.9)  # reject Kv near upper bound

if __name__ == '__main__':
    multiprocessing.freeze_support()

    DAY1_LOGS = sorted([
        f for f in globmod.glob(os.path.join(LOG_DIR, 'akit_26-03-07_*.wpilog'))
        if '_sim' not in f
    ])

    print(f"Using {len(DAY1_LOGS)} day-1 logs, modules: {MODULES}")
    for log in DAY1_LOGS:
        print(f"  {os.path.basename(log)}")

    all_segments = load_segments(DAY1_LOGS)
    print(f"\nLoaded {len(all_segments)} module-match segments")

    # Build work list with pre-filtering
    work = []
    skipped = 0
    for t_unif, v_unif, V_unif, mod, logname in all_segments:
        active = np.abs(V_unif) > 0.2
        for start in range(0, len(t_unif)-WINDOW, STRIDE):
            end = start + WINDOW
            v_w = v_unif[start:end]
            V_w = V_unif[start:end]

            # Pre-filter: need actual dynamics, not just holding position
            if active[start:end].mean() < MIN_ACTIVE_FRAC:
                skipped += 1
                continue
            if v_w.std() < MIN_VEL_STD:
                skipped += 1
                continue
            if V_w.std() < MIN_VOLT_STD:
                skipped += 1
                continue

            t_w = t_unif[start:end]
            work.append((t_w, V_w, v_w, mod, logname))

    n_cpus = multiprocessing.cpu_count()
    print(f"Fitting {len(work)} windows on {n_cpus} cores (skipped {skipped} low-activity windows)")

    with ProcessPoolExecutor(max_workers=n_cpus) as pool:
        futures = {pool.submit(fit_one_window, w): i for i, w in enumerate(work)}
        fit_results = [None] * len(work)
        done = 0
        for fut in as_completed(futures):
            idx = futures[fut]
            r = fut.result()
            fit_results[idx] = r
            done += 1
            tag = "*" if is_good_fit(r) else " "
            print(f"  [{done}/{len(work)}]{tag} {r['mod']} t={r['t0']:.0f}s  "
                  f"tau={r['tau']*1000:.1f}ms Kv={r['Kv']:.3f} R2={r['r2']:.3f} "
                  f"v_std={r['v_std']:.2f}", flush=True)

    # ---- Post-filter ----
    good = [r for r in fit_results if is_good_fit(r)]
    bad = [r for r in fit_results if not is_good_fit(r)]
    print(f"\n{'='*60}")
    print(f"Total: {len(fit_results)} windows, {len(good)} good, {len(bad)} rejected")
    if bad:
        bad_reasons = []
        for r in bad:
            reasons = []
            if r['tau'] >= MAX_TAU: reasons.append('tau>=150ms')
            if r['r2'] <= MIN_R2: reasons.append(f"R2={r['r2']:.2f}")
            if r['Kv'] >= 2.9: reasons.append('Kv_bound')
            bad_reasons.append(', '.join(reasons) if reasons else 'boundary')
        from collections import Counter
        for reason, count in Counter(bad_reasons).most_common(5):
            print(f"  rejected: {reason} ({count}x)")

    if len(good) == 0:
        print("ERROR: no good fits!")
        exit(1)

    taus   = np.array([r['tau']   for r in good])
    Kvs    = np.array([r['Kv']    for r in good])
    delays = np.array([r['delay'] for r in good])
    kSs    = np.array([r['kS']    for r in good])
    rmses  = np.array([r['rmse']  for r in good])
    r2s    = np.array([r['r2']    for r in good])

    # Compute per-window kA for distribution analysis
    kAs = taus / Kvs

    print(f"\n=== PLANT PARAMETERS ({len(good)} good fits) ===")
    print(f"{'param':>8s}   {'median':>8s}  {'IQR_lo':>8s}  {'IQR_hi':>8s}  {'std':>8s}")
    for name, arr, unit in [
        ('tau_m', taus*1000, 'ms'),
        ('Kv', Kvs, '(m/s)/V'),
        ('delay', delays*1000, 'ms'),
        ('kS', kSs, 'm/s'),
    ]:
        print(f"{name:>8s}   {np.median(arr):8.3f}  {np.percentile(arr,25):8.3f}  "
              f"{np.percentile(arr,75):8.3f}  {arr.std():8.3f}  {unit}")

    print(f"\nR2:   median={np.median(r2s):.3f}  min={r2s.min():.3f}")
    print(f"RMSE: median={np.median(rmses):.3f} m/s")

    # Per-module breakdown
    print(f"\nPer-module tau_m (median, ms):")
    for mod in MODULES:
        mod_taus = np.array([r['tau'] for r in good if r['mod'] == mod])
        mod_Kvs = np.array([r['Kv'] for r in good if r['mod'] == mod])
        mod_r2 = np.array([r['r2'] for r in good if r['mod'] == mod])
        if len(mod_taus) > 0:
            mod_kAs = mod_taus / mod_Kvs
            print(f"  {mod}: tau={np.median(mod_taus)*1000:.1f}ms  "
                  f"Kv={np.median(mod_Kvs):.4f}  "
                  f"kA={np.median(mod_kAs)*1000:.2f}ms/(m/s/V)  "
                  f"n={len(mod_taus)}  R2={np.median(mod_r2):.3f}")

    # ---- FRC FEEDFORWARD CONSTANTS ----
    tau_med = np.median(taus)
    Kv_med  = np.median(Kvs)
    kS_med  = np.median(kSs)
    kA_arr  = taus / Kvs

    kV_frc = 1.0 / Kv_med
    kA_frc = np.median(kA_arr)  # median of per-window kA, not ratio of medians
    kS_frc = kS_med / Kv_med

    print(f"\n=== FRC FEEDFORWARD CONSTANTS ===")
    print(f"kS = {kS_frc:.6f} V")
    print(f"kV = {kV_frc:.6f} V/(m/s)")
    print(f"kA = {kA_frc:.6f} V/(m/s^2)")
    print(f"     kA distribution: median={np.median(kA_arr)*1000:.3f}  "
          f"IQR=[{np.percentile(kA_arr,25)*1000:.3f}, {np.percentile(kA_arr,75)*1000:.3f}] mV/(m/s^2)")
    print(f"\n  Current code constants (for comparison):")
    print(f"    DRIVE_kS = 0.093052 V")
    print(f"    DRIVE_kV = 1.8968 V/(m/s)   =>  Kv = {1/1.8968:.4f} (m/s)/V")
    print(f"    DRIVE_kA = 0.015 V/(m/s^2)")

    # ---- UNCERTAINTY ----
    n = len(good)
    print(f"\n=== UNCERTAINTY (n={n}) ===")
    print(f"tau_m: {tau_med*1000:.1f} ms  IQR=[{np.percentile(taus,25)*1000:.1f}, {np.percentile(taus,75)*1000:.1f}]  "
          f"SEM={taus.std()/np.sqrt(n)*1000:.2f} ms")
    print(f"Kv:    {Kv_med:.4f}       IQR=[{np.percentile(Kvs,25):.4f}, {np.percentile(Kvs,75):.4f}]  "
          f"SEM={Kvs.std()/np.sqrt(n):.5f}")
    print(f"kA:    {kA_frc*1000:.2f} mV/(m/s2)  IQR=[{np.percentile(kA_arr,25)*1000:.2f}, {np.percentile(kA_arr,75)*1000:.2f}]  "
          f"SEM={kA_arr.std()/np.sqrt(n)*1000:.3f}")

    # ---- DRIVE ERROR ANALYSIS ----
    # Also load setpoint/target velocities and drive error for correlation
    print(f"\n=== DRIVE ERROR ANALYSIS ===")
    for log_path in DAY1_LOGS:
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
                    if name == f'/Swerve/{mod}/DriveVelMetersPerSec':
                        data.setdefault((mod, 'vel'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))
                    if name == f'/RealOutputs/Swerve/{mod}/setpointVelMetersPerSec':
                        data.setdefault((mod, 'setpoint'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))
                    if name == f'/RealOutputs/Swerve/{mod}/targetVelMetersPerSec':
                        data.setdefault((mod, 'target'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))
                    if name == f'/RealOutputs/Swerve/{mod}/driveFFVolts':
                        data.setdefault((mod, 'ff'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))

        logname = os.path.basename(log_path)
        for mod in MODULES:
            vel = data.get((mod, 'vel'), [])
            sp = data.get((mod, 'setpoint'), [])
            if len(vel) < 100 or len(sp) < 100:
                continue
            vel_arr = np.array(vel)
            sp_arr = np.array(sp)
            t0 = max(vel_arr[0, 0], sp_arr[0, 0])
            t1 = min(vel_arr[-1, 0], sp_arr[-1, 0])
            t = np.arange(t0, t1, 0.02)
            vel_interp = np.interp(t, vel_arr[:, 0], vel_arr[:, 1])
            sp_interp = np.interp(t, sp_arr[:, 0], sp_arr[:, 1])
            err = vel_interp - sp_interp
            # Only look at times when actually moving
            moving = np.abs(sp_interp) > 0.5
            if moving.sum() < 50:
                continue
            err_moving = err[moving]
            accel = np.gradient(sp_interp, 0.02)
            high_accel = moving & (np.abs(accel) > 2.0)  # m/s^2
            low_accel = moving & (np.abs(accel) < 0.5)
            print(f"  {logname[-30:]} {mod}: "
                  f"err_rms={np.sqrt(np.mean(err_moving**2)):.3f} m/s  "
                  f"err_p95={np.percentile(np.abs(err_moving), 95):.3f} m/s  "
                  f"err_max={np.max(np.abs(err_moving)):.3f} m/s  "
                  f"moving={moving.mean()*100:.0f}%")
            if high_accel.any() and low_accel.any():
                err_hi = np.sqrt(np.mean(err[high_accel]**2))
                err_lo = np.sqrt(np.mean(err[low_accel]**2))
                print(f"    hi_accel_err={err_hi:.3f}  lo_accel_err={err_lo:.3f}  "
                      f"ratio={err_hi/max(err_lo,1e-6):.2f}x  "
                      f"hi_accel_frac={high_accel.mean()*100:.0f}%")

    # ---- COMPARISON WITH CURRENT CONSTANTS ----
    print(f"\n=== RECOMMENDATION ===")
    print(f"Fitted:  kS={kS_frc:.6f}  kV={kV_frc:.6f}  kA={kA_frc:.6f}")
    print(f"Current: kS=0.093052     kV=1.896800     kA=0.015000")
    print(f"")
    print(f"kS diff: {abs(kS_frc - 0.093052):.6f} V  ({(kS_frc/0.093052 - 1)*100:+.1f}%)")
    print(f"kV diff: {abs(kV_frc - 1.8968):.6f} V/(m/s)  ({(kV_frc/1.8968 - 1)*100:+.1f}%)")
    print(f"kA diff: {abs(kA_frc - 0.015):.6f} V/(m/s^2)  ({(kA_frc/0.015 - 1)*100:+.1f}%)")
    print(f"")
    print(f"tau_m = {tau_med*1000:.1f} ms (drive motor + wheel inertia time constant)")
    print(f"  For comparison, turn tau_m was ~20.5 ms")
