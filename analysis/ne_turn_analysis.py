"""Turn motor response timescale analysis from day 1 match data (NE, NW, SE).

Plant model: tau*dw/dt = Kv*V(t-delay) - w - kS*sign(w)
FRC convention: V = kS*sign(w) + kV*w + kA*dw/dt
  where kV = 1/Kv, kA = tau/Kv, kS_frc = kS/Kv
"""
from wpiutil.log import DataLogReader
import numpy as np
from scipy.optimize import differential_evolution
from scipy.linalg import solve_continuous_are
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
MIN_VEL_STD = 1.0       # rad/s — reject windows with barely any motion
MIN_VOLT_STD = 0.5      # V — reject windows with nearly constant voltage

# ---- Post-filter thresholds (for "good" fits) ----
MAX_TAU = 0.08           # 80ms — above this, fit is degenerate
MIN_R2 = 0.5
MAX_TAU_BOUNDARY = 0.48  # reject fits near the upper bound (0.5)

# ---- PLANT MODEL ----
def simulate_plant(params, t_arr, V_arr, w0):
    tau_m, Kv, delay, kS = params
    n = len(t_arr)
    w = np.empty(n); w[0] = w0
    delay_steps = max(0, int(round(delay / dt_u)))
    for i in range(n-1):
        V_d = V_arr[max(0, i - delay_steps)]
        dwdt = (Kv * V_d - w[i] - kS * np.sign(w[i])) / tau_m
        w[i+1] = w[i] + dt_u * dwdt
        if abs(w[i+1]) > 1e4:
            w[i+1:] = 1e4
            break
    return w

def cost(params, t_arr, V_arr, w_meas):
    tau_m, Kv, delay, kS = params
    if tau_m < 0.005 or Kv < 0.3:
        return 1e9
    w_sim = simulate_plant(params, t_arr, V_arr, w_meas[0])
    return np.mean((w_sim - w_meas)**2)

def fit_one_window(args):
    """Fit a single window -- runs in worker process."""
    t_w, V_w, w_w, mod, logname = args
    res = differential_evolution(
        cost, args=(t_w, V_w, w_w),
        bounds=[(0.005, 0.5), (0.5, 15.0), (0.0, 0.08), (0.0, 5.0)],
        seed=42, maxiter=80, tol=1e-5, workers=1, popsize=10
    )
    tau_f, Kv_f, del_f, kS_f = res.x
    rmse = np.sqrt(res.fun)
    w_var = w_w.var()
    r2 = 1 - res.fun / w_var if w_var > 0 else -999
    return {
        't0': t_w[0], 'tau': tau_f, 'Kv': Kv_f, 'delay': del_f,
        'kS': kS_f, 'rmse': rmse, 'w_std': w_w.std(), 'V_std': V_w.std(),
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
                    if name == f'/Swerve/{mod}/TurnVelRadiansPerSec':
                        data.setdefault((mod, 'vel'), []).append(
                            (record.getTimestamp()/1e6, record.getDouble()))
                    elif name == f'/Swerve/{mod}/TurnVoltageVolts':
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
            w_unif = np.interp(t_unif, t_vel, vel)
            V_unif = np.interp(t_unif, t_volt, volt)
            all_segments.append((t_unif, w_unif, V_unif, mod, os.path.basename(log_path)))
    return all_segments

def is_good_fit(r):
    """Post-filter: reject degenerate fits."""
    return (r['tau'] < MAX_TAU
            and r['tau'] < MAX_TAU_BOUNDARY
            and r['r2'] > MIN_R2
            and r['Kv'] < 14.0)  # reject Kv near upper bound

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
    for t_unif, w_unif, V_unif, mod, logname in all_segments:
        active = np.abs(V_unif) > 0.2
        for start in range(0, len(t_unif)-WINDOW, STRIDE):
            end = start + WINDOW
            w_w = w_unif[start:end]
            V_w = V_unif[start:end]

            # Pre-filter: need actual dynamics, not just holding position
            if active[start:end].mean() < MIN_ACTIVE_FRAC:
                skipped += 1
                continue
            if w_w.std() < MIN_VEL_STD:
                skipped += 1
                continue
            if V_w.std() < MIN_VOLT_STD:
                skipped += 1
                continue

            t_w = t_unif[start:end]
            work.append((t_w, V_w, w_w, mod, logname))

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
                  f"tau={r['tau']*1000:.1f}ms Kv={r['Kv']:.2f} R2={r['r2']:.3f} "
                  f"w_std={r['w_std']:.1f}", flush=True)

    # ---- Post-filter ----
    good = [r for r in fit_results if is_good_fit(r)]
    bad = [r for r in fit_results if not is_good_fit(r)]
    print(f"\n{'='*60}")
    print(f"Total: {len(fit_results)} windows, {len(good)} good, {len(bad)} rejected")
    if bad:
        bad_reasons = []
        for r in bad:
            reasons = []
            if r['tau'] >= MAX_TAU: reasons.append('tau>=80ms')
            if r['r2'] <= MIN_R2: reasons.append(f"R2={r['r2']:.2f}")
            if r['Kv'] >= 14.0: reasons.append('Kv_bound')
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
        ('Kv', Kvs, 'rad/s/V'),
        ('delay', delays*1000, 'ms'),
        ('kS', kSs, 'rad/s'),
    ]:
        print(f"{name:>8s}   {np.median(arr):8.3f}  {np.percentile(arr,25):8.3f}  "
              f"{np.percentile(arr,75):8.3f}  {arr.std():8.3f}  {unit}")

    print(f"\nR2:   median={np.median(r2s):.3f}  min={r2s.min():.3f}")
    print(f"RMSE: median={np.median(rmses):.3f} rad/s")

    # Per-module breakdown
    print(f"\nPer-module tau_m (median, ms):")
    for mod in MODULES:
        mod_taus = np.array([r['tau'] for r in good if r['mod'] == mod])
        mod_Kvs = np.array([r['Kv'] for r in good if r['mod'] == mod])
        mod_r2 = np.array([r['r2'] for r in good if r['mod'] == mod])
        if len(mod_taus) > 0:
            mod_kAs = mod_taus / mod_Kvs
            print(f"  {mod}: tau={np.median(mod_taus)*1000:.1f}ms  "
                  f"Kv={np.median(mod_Kvs):.3f}  "
                  f"kA={np.median(mod_kAs)*1000:.2f}mV/(rad/s2)  "
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
    print(f"kS = {kS_frc:.5f} V")
    print(f"kV = {kV_frc:.5f} V/(rad/s)")
    print(f"kA = {kA_frc:.6f} V/(rad/s^2)")
    print(f"     kA distribution: median={np.median(kA_arr)*1000:.3f}  "
          f"IQR=[{np.percentile(kA_arr,25)*1000:.3f}, {np.percentile(kA_arr,75)*1000:.3f}] mV/(rad/s^2)")
    print(f"     (old swerve for reference: kS=0.194, kV=0.366, kA=0.044)")

    # ---- UNCERTAINTY ----
    n = len(good)
    print(f"\n=== UNCERTAINTY (n={n}) ===")
    print(f"tau_m: {tau_med*1000:.1f} ms  IQR=[{np.percentile(taus,25)*1000:.1f}, {np.percentile(taus,75)*1000:.1f}]  "
          f"SEM={taus.std()/np.sqrt(n)*1000:.2f} ms")
    print(f"Kv:    {Kv_med:.3f}       IQR=[{np.percentile(Kvs,25):.3f}, {np.percentile(Kvs,75):.3f}]  "
          f"SEM={Kvs.std()/np.sqrt(n):.4f}")
    print(f"kA:    {kA_frc*1000:.2f} mV/(rad/s2)  IQR=[{np.percentile(kA_arr,25)*1000:.2f}, {np.percentile(kA_arr,75)*1000:.2f}]  "
          f"SEM={kA_arr.std()/np.sqrt(n)*1000:.3f}")

    # ---- LQR GAINS ----
    tau = tau_med
    Kv = Kv_med

    A = np.array([[0, -1],
                  [0, -1/tau]])
    B = np.array([[0],
                  [Kv/tau]])

    print(f"\n=== LQR GAINS ===")
    print(f"Plant: tau={tau*1000:.1f}ms  Kv={Kv:.3f} rad/s/V")
    print(f"  A = [[0, -1], [0, {-1/tau:.1f}]]  B = [[0], [{Kv/tau:.1f}]]")
    print()

    configs = [
        ("Conservative", np.diag([100, 1]), np.array([[1.0]])),
        ("Balanced",     np.diag([400, 1]), np.array([[0.5]])),
        ("Aggressive",   np.diag([1600, 4]), np.array([[0.25]])),
    ]

    for label, Q, R in configs:
        P = solve_continuous_are(A, B, Q, R)
        K_lqr = np.linalg.inv(R) @ B.T @ P
        kP_actual = -K_lqr[0, 0]
        kD_actual = K_lqr[0, 1]

        Acl = A - B @ K_lqr
        eigs = np.linalg.eigvals(Acl)
        settling = 4.0 / (-eigs.real.max())

        print(f"  {label} (Q=diag({Q[0,0]:.0f},{Q[1,1]:.0f}), R={R[0,0]:.2f}):")
        print(f"    kP={kP_actual:.2f}  kD={kD_actual:.4f}  settle={settling*1000:.0f}ms  "
              f"poles={eigs[0]:.1f}, {eigs[1]:.1f}")

    # Current gains
    print()
    K_current = np.array([[-20.0, 0.4]])
    Acl_cur = A - B @ K_current
    eigs_cur = np.linalg.eigvals(Acl_cur)
    if eigs_cur.real.max() < 0:
        settle_cur = 4.0 / (-eigs_cur.real.max())
        print(f"  Current (kP=20, kD=0.4):  settle={settle_cur*1000:.0f}ms  "
              f"poles={eigs_cur[0]:.1f}, {eigs_cur[1]:.1f}")
    else:
        print(f"  Current (kP=20, kD=0.4):  UNSTABLE  "
              f"poles={eigs_cur[0]:.1f}, {eigs_cur[1]:.1f}")
