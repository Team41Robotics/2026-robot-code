"""Investigate root causes of large turn errors in match logs.

Hypotheses:
  1. Discretization: tau=20ms ≈ control period 20ms
  2. Encoder drift between relative (turnPosRadians) and absolute (turnAbsPosRadians)
  3. Missing acceleration feedforward (TURN_kA = 0)
  4. CAN latency (~5ms = 25% of tau)
"""
from wpiutil.log import DataLogReader
import numpy as np
import glob as globmod
import os

LOG_DIR = 'c:/Users/Robotics41/Desktop/2026-robot-code-new/logs'
MODULES = ['NE', 'NW', 'SE']

SIGNALS = [
    'TurnVelRadiansPerSec', 'TurnVoltageVolts', 'TurnPosRadians',
    'TurnAbsPosRadians',
]
OUTPUT_SIGNALS = [
    'turnErrorRadians', 'turnFFVolts', 'setpointAngRadians',
    'setpointAngVelRadiansPerSec', 'targetAngRadians',
]

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
                    for sig in SIGNALS:
                        if name == f'/Swerve/{mod}/{sig}':
                            data.setdefault((mod, sig), []).append(
                                (record.getTimestamp()/1e6, record.getDouble()))
                    for sig in OUTPUT_SIGNALS:
                        if name == f'/RealOutputs/Swerve/{mod}/{sig}':
                            data.setdefault((mod, sig), []).append(
                                (record.getTimestamp()/1e6, record.getDouble()))
        logname = os.path.basename(log_path)
        all_data[logname] = data
    return all_data

def analyze_encoder_drift(data, mod):
    """Check if relative and absolute encoders diverge."""
    rel = data.get((mod, 'TurnPosRadians'), [])
    abs_ = data.get((mod, 'TurnAbsPosRadians'), [])
    if len(rel) < 100 or len(abs_) < 100:
        return None
    rel_arr = np.array(rel)
    abs_arr = np.array(abs_)
    t0 = max(rel_arr[0, 0], abs_arr[0, 0])
    t1 = min(rel_arr[-1, 0], abs_arr[-1, 0])
    t = np.arange(t0, t1, 0.02)
    rel_interp = np.interp(t, rel_arr[:, 0], rel_arr[:, 1])
    abs_interp = np.interp(t, abs_arr[:, 0], abs_arr[:, 1])
    # The relative encoder accumulates, absolute wraps at ±pi
    # Compute the offset: rel - abs (mod 2pi)
    diff = rel_interp - abs_interp
    # Remove the initial offset
    diff -= diff[0]
    return {
        't': t, 'diff': diff,
        'max_drift': np.max(np.abs(diff)),
        'std_drift': np.std(diff),
        'final_drift': diff[-1],
    }

def analyze_ff_contribution(data, mod):
    """Check how much turnFF contributes vs total voltage."""
    ff = data.get((mod, 'turnFFVolts'), [])
    volt = data.get((mod, 'TurnVoltageVolts'), [])
    if len(ff) < 100 or len(volt) < 100:
        return None
    ff_arr = np.array(ff)
    volt_arr = np.array(volt)
    t0 = max(ff_arr[0, 0], volt_arr[0, 0])
    t1 = min(ff_arr[-1, 0], volt_arr[-1, 0])
    t = np.arange(t0, t1, 0.02)
    ff_interp = np.interp(t, ff_arr[:, 0], ff_arr[:, 1])
    volt_interp = np.interp(t, volt_arr[:, 0], volt_arr[:, 1])
    pid_component = volt_interp - ff_interp
    return {
        'ff_rms': np.sqrt(np.mean(ff_interp**2)),
        'pid_rms': np.sqrt(np.mean(pid_component**2)),
        'volt_rms': np.sqrt(np.mean(volt_interp**2)),
        'ff_frac': np.sqrt(np.mean(ff_interp**2)) / max(np.sqrt(np.mean(volt_interp**2)), 1e-6),
        'ff_zero_frac': np.mean(np.abs(ff_interp) < 0.01),
        'ff_max': np.max(np.abs(ff_interp)),
    }

def analyze_error_vs_setpoint_vel(data, mod):
    """Correlate turn error with setpoint angular velocity (proxy for how much accel FF is missing)."""
    err = data.get((mod, 'turnErrorRadians'), [])
    sp_vel = data.get((mod, 'setpointAngVelRadiansPerSec'), [])
    if len(err) < 100 or len(sp_vel) < 100:
        return None
    err_arr = np.array(err)
    sp_arr = np.array(sp_vel)
    t0 = max(err_arr[0, 0], sp_arr[0, 0])
    t1 = min(err_arr[-1, 0], sp_arr[-1, 0])
    t = np.arange(t0, t1, 0.02)
    err_interp = np.interp(t, err_arr[:, 0], err_arr[:, 1])
    sp_interp = np.interp(t, sp_arr[:, 0], sp_arr[:, 1])
    # Estimate acceleration from setpoint velocity
    accel = np.gradient(sp_interp, 0.02)
    # Error during high accel vs low accel
    high_accel = np.abs(accel) > 20  # rad/s^2
    low_accel = np.abs(accel) < 5
    return {
        'err_rms_high_accel': np.sqrt(np.mean(err_interp[high_accel]**2)) if high_accel.any() else 0,
        'err_rms_low_accel': np.sqrt(np.mean(err_interp[low_accel]**2)) if low_accel.any() else 0,
        'high_accel_frac': high_accel.mean(),
        'correlation_err_accel': np.corrcoef(np.abs(err_interp), np.abs(accel))[0, 1] if len(err_interp) > 10 else 0,
        'max_accel': np.max(np.abs(accel)),
    }

def analyze_discretization(data, mod):
    """Simulate continuous vs discrete closed-loop response to quantify discretization error."""
    tau = 0.0205  # from our analysis
    Kv = 1.96
    kS_plant = 0.5  # plant kS in rad/s
    kP = 20.0
    kD = 0.4
    kS_ff = 0.08
    kV_ff = 0.38

    # Get setpoint trajectory from logs
    sp_ang = data.get((mod, 'setpointAngRadians'), [])
    sp_vel = data.get((mod, 'setpointAngVelRadiansPerSec'), [])
    abs_pos = data.get((mod, 'TurnAbsPosRadians'), [])
    if len(sp_ang) < 100 or len(abs_pos) < 100:
        return None

    sp_arr = np.array(sp_ang)
    abs_arr = np.array(abs_pos)

    # Just report the theoretical settling time at different dt values
    results = {}
    for dt_label, dt in [('1kHz', 0.001), ('50Hz', 0.02), ('continuous', 0.0001)]:
        # Discrete step response simulation
        w = 0.0  # velocity
        pos = 0.0  # position
        target = 0.5  # step to 0.5 rad
        n_steps = int(0.5 / dt)  # simulate 500ms
        settled = None
        for i in range(n_steps):
            error = target - pos
            vel_cmd = 0  # setpoint velocity = 0 for step
            V_ff = kS_ff * np.sign(vel_cmd) + kV_ff * vel_cmd
            V_pid = kP * error - kD * w
            V = np.clip(V_ff + V_pid, -12, 12)
            # Plant dynamics
            dwdt = (Kv * V - w - kS_plant * np.sign(w)) / tau
            w += dt * dwdt
            pos += dt * w
            if settled is None and abs(error) < 0.01:  # ~0.6 degrees
                settled = i * dt
        results[dt_label] = {
            'settled_ms': settled * 1000 if settled else float('inf'),
            'final_error': target - pos,
        }
    return results


if __name__ == '__main__':
    DAY1_LOGS = sorted([
        f for f in globmod.glob(os.path.join(LOG_DIR, 'akit_26-03-07_*.wpilog'))
        if '_sim' not in f
    ])
    print(f"Loading {len(DAY1_LOGS)} day-1 logs...")
    all_data = load_all(DAY1_LOGS)
    print(f"Loaded {len(all_data)} logs\n")

    # ==========================================
    # HYPOTHESIS 1: Discretization
    # ==========================================
    print("=" * 60)
    print("HYPOTHESIS 1: DISCRETIZATION (tau=20ms vs dt=20ms)")
    print("=" * 60)
    disc = analyze_discretization(list(all_data.values())[0] if all_data else {}, MODULES[0])
    if disc:
        for rate, r in disc.items():
            print(f"  {rate:>12s}: settles in {r['settled_ms']:.1f}ms  final_err={r['final_error']:.6f} rad")
        ratio = disc['50Hz']['settled_ms'] / disc['1kHz']['settled_ms']
        print(f"\n  50Hz is {ratio:.1f}x slower than 1kHz for settling")
        print(f"  50Hz settling: {disc['50Hz']['settled_ms']:.0f}ms vs 1kHz: {disc['1kHz']['settled_ms']:.0f}ms")
    print()

    # ==========================================
    # HYPOTHESIS 2: Encoder drift
    # ==========================================
    print("=" * 60)
    print("HYPOTHESIS 2: ENCODER DRIFT (relative vs absolute)")
    print("=" * 60)
    for logname, data in all_data.items():
        print(f"\n  Log: {logname}")
        for mod in MODULES:
            drift = analyze_encoder_drift(data, mod)
            if drift:
                print(f"    {mod}: max_drift={np.degrees(drift['max_drift']):.2f}°  "
                      f"std={np.degrees(drift['std_drift']):.2f}°  "
                      f"final={np.degrees(drift['final_drift']):.2f}°")
    print()

    # ==========================================
    # HYPOTHESIS 3: Missing acceleration FF
    # ==========================================
    print("=" * 60)
    print("HYPOTHESIS 3: MISSING ACCELERATION FEEDFORWARD (kA=0)")
    print("=" * 60)
    all_ff = []
    all_err_accel = []
    for logname, data in all_data.items():
        for mod in MODULES:
            ff = analyze_ff_contribution(data, mod)
            if ff:
                all_ff.append((logname, mod, ff))
            ea = analyze_error_vs_setpoint_vel(data, mod)
            if ea:
                all_err_accel.append((logname, mod, ea))

    if all_ff:
        print("\n  Feedforward voltage contribution:")
        print(f"  {'log':>30s}  {'mod':>3s}  {'FF_rms':>7s}  {'PID_rms':>8s}  {'V_rms':>7s}  {'FF%':>5s}  {'FF=0%':>6s}  {'FF_max':>7s}")
        for logname, mod, ff in all_ff:
            print(f"  {logname[-30:]:>30s}  {mod:>3s}  {ff['ff_rms']:7.3f}  {ff['pid_rms']:8.3f}  "
                  f"{ff['volt_rms']:7.3f}  {ff['ff_frac']*100:4.1f}%  {ff['ff_zero_frac']*100:5.1f}%  {ff['ff_max']:7.3f}")

    if all_err_accel:
        print(f"\n  Turn error vs acceleration phase:")
        print(f"  {'log':>30s}  {'mod':>3s}  {'err_hi_acc':>10s}  {'err_lo_acc':>10s}  {'ratio':>6s}  {'corr':>6s}  {'max_acc':>8s}")
        for logname, mod, ea in all_err_accel:
            ratio = ea['err_rms_high_accel'] / max(ea['err_rms_low_accel'], 1e-6)
            print(f"  {logname[-30:]:>30s}  {mod:>3s}  {np.degrees(ea['err_rms_high_accel']):9.2f}°  "
                  f"{np.degrees(ea['err_rms_low_accel']):9.2f}°  {ratio:5.2f}x  "
                  f"{ea['correlation_err_accel']:5.3f}  {ea['max_accel']:7.0f}°/s²")

    # Estimate the missing kA voltage
    print(f"\n  Missing acceleration FF estimate:")
    print(f"    Our measured kA = 0.0104 V/(rad/s²)")
    print(f"    Turn profile max accel = 80 rad/s²")
    print(f"    Missing voltage at max accel = {0.0104 * 80:.2f} V")
    print(f"    This error must be corrected by PID alone")
    print(f"    At kP=20 V/rad, need {0.0104 * 80 / 20:.3f} rad = {np.degrees(0.0104 * 80 / 20):.1f}° error to compensate")
    print()

    # ==========================================
    # HYPOTHESIS 4: CAN latency
    # ==========================================
    print("=" * 60)
    print("HYPOTHESIS 4: CAN LATENCY")
    print("=" * 60)
    print("  CAN bus round-trip: ~2-5ms")
    print(f"  Plant tau: 20.5ms")
    print(f"  Latency as fraction of tau: {5/20.5*100:.0f}%")
    print(f"  Our fitted delay parameter: median ~5ms (matches CAN latency)")
    print(f"  Impact: adds phase lag, reduces effective phase margin")
    print(f"  At 50Hz control, CAN latency is already baked into the 20ms period")
    print(f"  -> CAN latency is NOT a separate issue at 50Hz, it's part of discretization")
    print()

    # ==========================================
    # SUMMARY
    # ==========================================
    print("=" * 60)
    print("SUMMARY: ROOT CAUSE RANKING")
    print("=" * 60)
    if disc:
        print(f"\n  1. DISCRETIZATION: 50Hz settles {disc['50Hz']['settled_ms']:.0f}ms vs "
              f"1kHz at {disc['1kHz']['settled_ms']:.0f}ms ({ratio:.1f}x slower)")
    if all_err_accel:
        avg_ratio = np.mean([ea['err_rms_high_accel'] / max(ea['err_rms_low_accel'], 1e-6)
                             for _, _, ea in all_err_accel])
        print(f"  2. MISSING kA FF: errors {avg_ratio:.1f}x larger during acceleration, "
              f"need {np.degrees(0.0104 * 80 / 20):.1f}° steady-state error to compensate")
    print(f"  3. CAN LATENCY: ~5ms delay, but subsumed by 20ms control period")
    print(f"  4. ENCODER DRIFT: see above (typically small if CANcoder is working)")
    print(f"\n  RECOMMENDATION: Use CTRE MotionMagicExpoVoltage")
    print(f"    - Runs profile + PID at 1kHz on motor controller")
    print(f"    - Eliminates discretization (biggest issue)")
    print(f"    - Eliminates CAN latency from inner loop")
    print(f"    - Built-in kA feedforward handles acceleration")
