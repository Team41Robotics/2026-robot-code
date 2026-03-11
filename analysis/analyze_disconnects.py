"""Analyze disconnects: CAN devices, cameras, IMU, radio."""
import sys, os
from collections import defaultdict
from wpiutil.log import DataLogReader

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

def read_int64s(reader, keys):
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
                    val = record.getInteger()
                    ts = record.getTimestamp() / 1e6
                    data[entries[eid]].append((ts, val))
                except Exception:
                    pass
    return data

def find_enabled_periods(enabled_series):
    periods = []
    start = None
    for t, v in enabled_series:
        if v and start is None:
            start = t
        elif not v and start is not None:
            periods.append((start, t))
            start = None
    if start is not None:
        periods.append((start, enabled_series[-1][0]))
    return periods

def find_disconnects(series, name):
    """Find transitions from connected to disconnected and vice versa."""
    events = []
    for i in range(1, len(series)):
        prev_t, prev_v = series[i-1]
        curr_t, curr_v = series[i]
        if prev_v and not curr_v:
            events.append((curr_t, "DISCONNECTED"))
        elif not prev_v and curr_v:
            events.append((curr_t, "RECONNECTED"))
    return events

def find_stale_signals(series, name, max_gap_s=0.5):
    """Find gaps in data where no updates came for longer than max_gap_s."""
    gaps = []
    for i in range(1, len(series)):
        dt = series[i][0] - series[i-1][0]
        if dt > max_gap_s:
            gaps.append((series[i-1][0], series[i][0], dt))
    return gaps

def analyze_file(path):
    print(f"\n{'='*90}")
    print(f"  DISCONNECTS: {os.path.basename(path)}")
    print(f"{'='*90}")

    # Get enabled periods
    reader = DataLogReader(path)
    en_data = read_booleans(reader, {"/DriverStation/Enabled"})
    enabled = en_data.get("/DriverStation/Enabled", [])
    enabled_periods = find_enabled_periods(enabled)
    if enabled_periods:
        match_start = enabled_periods[0][0]
        match_end = enabled_periods[-1][1]
        total_en = sum(e - s for s, e in enabled_periods)
        print(f"  Match: {match_start:.1f}s - {match_end:.1f}s (enabled {total_en:.1f}s)")
    else:
        match_start = 0
        match_end = float('inf')

    # --- Camera disconnects ---
    cam_bool_keys = {
        "//Vision/DuckyNE/IsConnected",
        "//Vision/DuckySE/IsConnected",
    }
    reader = DataLogReader(path)
    cam_b = read_booleans(reader, cam_bool_keys)

    print("\n--- CAMERAS ---")
    for cam in ["DuckyNE", "DuckySE"]:
        series = cam_b.get(f"//Vision/{cam}/IsConnected", [])
        if not series:
            print(f"  {cam}: NO DATA")
            continue

        # Overall connectivity
        total = len(series)
        connected = sum(1 for _, v in series if v)
        pct = 100 * connected / total if total > 0 else 0
        print(f"  {cam}: {pct:.1f}% connected ({connected}/{total} samples)")

        # Initial state
        if series:
            print(f"    Initial state: {'CONNECTED' if series[0][1] else 'DISCONNECTED'} at t={series[0][0]:.2f}s")

        # Disconnect/reconnect events
        events = find_disconnects(series, cam)
        if events:
            print(f"    {len(events)} state changes:")
            for t, evt in events:
                in_match = any(s <= t <= e for s, e in enabled_periods)
                flag = " ** DURING MATCH **" if in_match else ""
                print(f"      t={t:.2f}s: {evt}{flag}")
        else:
            print(f"    No state changes (stable {'connected' if series[0][1] else 'disconnected'})")

        # During enabled periods specifically
        en_samples = []
        for s, e in enabled_periods:
            en_samples.extend([(t, v) for t, v in series if s <= t <= e])
        if en_samples:
            en_connected = sum(1 for _, v in en_samples if v)
            en_pct = 100 * en_connected / len(en_samples)
            if en_pct < 100:
                print(f"    ** During match: {en_pct:.1f}% connected ({en_connected}/{len(en_samples)}) **")

        # Check for data gaps (stale camera data might indicate USB issues)
        gaps = find_stale_signals(series, cam, max_gap_s=2.0)
        match_gaps = [(s, e, d) for s, e, d in gaps
                      if any(es <= s <= ee or es <= e <= ee for es, ee in enabled_periods)]
        if match_gaps:
            print(f"    {len(match_gaps)} data gaps >2s during match:")
            for gs, ge, gd in match_gaps[:5]:
                print(f"      t={gs:.2f}s - {ge:.2f}s ({gd:.1f}s gap)")

    # --- IMU ---
    imu_bool_keys = {
        "//IMU/IsConnected",
        "//IMU/IsCalibrating",
    }
    reader = DataLogReader(path)
    imu_b = read_booleans(reader, imu_bool_keys)

    print("\n--- IMU ---")
    imu_conn = imu_b.get("//IMU/IsConnected", [])
    imu_cal = imu_b.get("//IMU/IsCalibrating", [])

    if imu_conn:
        total = len(imu_conn)
        connected = sum(1 for _, v in imu_conn if v)
        pct = 100 * connected / total
        if pct < 100:
            print(f"  Connected: {pct:.1f}% ({connected}/{total})")
            events = find_disconnects(imu_conn, "IMU")
            for t, evt in events:
                in_match = any(s <= t <= e for s, e in enabled_periods)
                flag = " ** DURING MATCH **" if in_match else ""
                print(f"    t={t:.2f}s: {evt}{flag}")
        else:
            print(f"  Connected: 100%")

        # Check for data gaps
        gaps = find_stale_signals(imu_conn, "IMU", max_gap_s=1.0)
        match_gaps = [(s, e, d) for s, e, d in gaps
                      if any(es <= s <= ee or es <= e <= ee for es, ee in enabled_periods)]
        if match_gaps:
            print(f"  ** {len(match_gaps)} IMU data gaps >1s during match **")
            for gs, ge, gd in match_gaps[:3]:
                print(f"    t={gs:.2f}s - {ge:.2f}s ({gd:.1f}s gap)")

    if imu_cal:
        cal_during_match = [(t, v) for t, v in imu_cal if v
                           and any(s <= t <= e for s, e in enabled_periods)]
        if cal_during_match:
            print(f"  ** IMU CALIBRATING during match: {len(cal_during_match)} samples **")
            print(f"    First: t={cal_during_match[0][0]:.2f}s")

    # --- CAN bus health ---
    can_int_keys = {
        "/SystemStats/CANBus/OffCount",
        "/SystemStats/CANBus/ReceiveErrorCount",
        "/SystemStats/CANBus/TransmitErrorCount",
        "/SystemStats/CANBus/TxFullCount",
    }
    reader = DataLogReader(path)
    can_i = read_int64s(reader, can_int_keys)

    print("\n--- CAN BUS ---")
    for key_name, label in [
        ("/SystemStats/CANBus/OffCount", "Bus-Off"),
        ("/SystemStats/CANBus/ReceiveErrorCount", "RX Errors"),
        ("/SystemStats/CANBus/TransmitErrorCount", "TX Errors"),
        ("/SystemStats/CANBus/TxFullCount", "TX Buffer Full"),
    ]:
        series = can_i.get(key_name, [])
        if not series:
            continue
        total_delta = series[-1][1] - series[0][1]

        # Check increments during match
        match_delta = 0
        for s, e in enabled_periods:
            in_range = [(t, v) for t, v in series if s <= t <= e]
            if len(in_range) >= 2:
                match_delta += in_range[-1][1] - in_range[0][1]

        if total_delta > 0 or match_delta > 0:
            flag = f" ({match_delta} during match)" if match_delta > 0 else ""
            print(f"  {label}: {total_delta} total{flag}")

            # Find when errors increment
            if total_delta > 0:
                increments = []
                for i in range(1, len(series)):
                    if series[i][1] > series[i-1][1]:
                        increments.append((series[i][0], series[i][1] - series[i-1][1]))
                if increments:
                    print(f"    Error timestamps:")
                    for t, cnt in increments[:10]:
                        in_match = any(s <= t <= e for s, e in enabled_periods)
                        flag = " ** DURING MATCH **" if in_match else ""
                        print(f"      t={t:.2f}s: +{cnt}{flag}")
        else:
            print(f"  {label}: 0")

    # --- Swerve motor disconnects (via bus voltage dropout) ---
    swerve_v_keys = set()
    for corner in ["NE", "NW", "SE", "SW"]:
        swerve_v_keys.add(f"/Swerve/{corner}/DriveBusVoltageVolts")
        swerve_v_keys.add(f"/Swerve/{corner}/TurnBusVoltageVolts")

    reader = DataLogReader(path)
    sw_v = read_doubles(reader, swerve_v_keys)

    print("\n--- SWERVE MOTOR CONNECTIVITY (bus voltage) ---")
    for corner in ["NE", "NW", "SE", "SW"]:
        drive_bv = sw_v.get(f"/Swerve/{corner}/DriveBusVoltageVolts", [])
        turn_bv = sw_v.get(f"/Swerve/{corner}/TurnBusVoltageVolts", [])

        issues = []

        for label, series in [("Drive", drive_bv), ("Turn", turn_bv)]:
            if not series:
                issues.append(f"{label}: NO DATA")
                continue

            # Check for 0V readings (disconnected motor)
            zero_readings = [(t, v) for t, v in series if v < 0.1]
            if zero_readings:
                # Filter to match time
                match_zeros = [(t, v) for t, v in zero_readings
                              if any(s <= t <= e for s, e in enabled_periods)]
                if match_zeros:
                    issues.append(f"{label}: {len(match_zeros)} zero-voltage samples during match (t={match_zeros[0][0]:.2f}s first)")

            # Check for data staleness
            match_samples = []
            for s, e in enabled_periods:
                match_samples.extend([(t, v) for t, v in series if s <= t <= e])
            if match_samples:
                gaps = find_stale_signals(match_samples, f"{corner}/{label}", max_gap_s=0.5)
                if gaps:
                    issues.append(f"{label}: {len(gaps)} data gaps >0.5s")

        if issues:
            print(f"  {corner}: " + "; ".join(issues))
        else:
            print(f"  {corner}: OK")

    # --- Shooter motor connectivity ---
    shooter_v_keys = {
        "//Shooter/FlywheelBusVoltageVolts",
        "//Shooter/TurretBusVoltageVolts",
        "//Shooter/HoodBusVoltageVolts",
    }
    reader = DataLogReader(path)
    sh_v = read_doubles(reader, shooter_v_keys)

    print("\n--- SHOOTER MOTOR CONNECTIVITY (bus voltage) ---")
    for key, label in [
        ("//Shooter/FlywheelBusVoltageVolts", "Flywheel"),
        ("//Shooter/TurretBusVoltageVolts", "Turret"),
        ("//Shooter/HoodBusVoltageVolts", "Hood"),
    ]:
        series = sh_v.get(key, [])
        if not series:
            print(f"  {label}: NO DATA")
            continue

        match_samples = []
        for s, e in enabled_periods:
            match_samples.extend([(t, v) for t, v in series if s <= t <= e])

        if not match_samples:
            print(f"  {label}: No data during match")
            continue

        zeros = [(t, v) for t, v in match_samples if v < 0.1]
        low = [(t, v) for t, v in match_samples if 0.1 <= v < 6.0]

        issues = []
        if zeros:
            issues.append(f"{len(zeros)} zero-voltage readings")
        if low:
            min_v = min(v for _, v in low)
            issues.append(f"{len(low)} readings <6V (min {min_v:.2f}V)")

        gaps = find_stale_signals(match_samples, label, max_gap_s=0.5)
        if gaps:
            issues.append(f"{len(gaps)} data gaps >0.5s")

        if issues:
            print(f"  {label}: " + "; ".join(issues))
        else:
            print(f"  {label}: OK")

    # --- Radio ---
    radio_bool_keys = {"/RadioStatus/Connected"}
    reader = DataLogReader(path)
    radio_b = read_booleans(reader, radio_bool_keys)

    radio_conn = radio_b.get("/RadioStatus/Connected", [])
    if radio_conn:
        print("\n--- RADIO ---")
        total = len(radio_conn)
        connected = sum(1 for _, v in radio_conn if v)
        pct = 100 * connected / total
        if pct < 100:
            print(f"  Connected: {pct:.1f}%")
            events = find_disconnects(radio_conn, "Radio")
            for t, evt in events[:10]:
                in_match = any(s <= t <= e for s, e in enabled_periods)
                flag = " ** DURING MATCH **" if in_match else ""
                print(f"    t={t:.2f}s: {evt}{flag}")
        else:
            print(f"  Connected: 100%")

    # --- DS/comms disconnects ---
    ds_bool_keys = {
        "/DriverStation/DSAttached",
        "/DriverStation/FMSAttached",
    }
    comms_int_keys = {
        "/SystemStats/CommsDisableCount",
    }
    reader = DataLogReader(path)
    ds_b = read_booleans(reader, ds_bool_keys)
    reader = DataLogReader(path)
    comms_i = read_int64s(reader, comms_int_keys)

    print("\n--- DRIVER STATION / COMMS ---")
    ds_attached = ds_b.get("/DriverStation/DSAttached", [])
    if ds_attached:
        ds_events = find_disconnects(ds_attached, "DS")
        match_ds = [(t, e) for t, e in ds_events if any(s <= t <= ee for s, ee in enabled_periods)]
        if match_ds:
            print(f"  ** {len(match_ds)} DS state changes during match **")
            for t, evt in match_ds:
                print(f"    t={t:.2f}s: {evt}")
        else:
            print(f"  DS: stable during match")

    comms_disable = comms_i.get("/SystemStats/CommsDisableCount", [])
    if comms_disable:
        total_disable = comms_disable[-1][1] - comms_disable[0][1]
        if total_disable > 0:
            match_disable = 0
            for s, e in enabled_periods:
                in_range = [(t, v) for t, v in comms_disable if s <= t <= e]
                if len(in_range) >= 2:
                    match_disable += in_range[-1][1] - in_range[0][1]
            print(f"  Comms disable count: {total_disable} total ({match_disable} during match)")
            if match_disable > 0:
                print(f"  ** COMMS LOST DURING MATCH {match_disable} times **")
        else:
            print(f"  No comms disables")

    # --- NT Client disconnects ---
    nt_bool_keys = set()
    reader = DataLogReader(path)
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if "NTClients" in d.name and "Connected" in d.name:
                nt_bool_keys.add(d.name)

    if nt_bool_keys:
        reader = DataLogReader(path)
        nt_b = read_booleans(reader, nt_bool_keys)

        print("\n--- NT CLIENTS ---")
        for key in sorted(nt_bool_keys):
            series = nt_b.get(key, [])
            if not series:
                continue
            client_name = key.split("/")[-2]
            events = find_disconnects(series, client_name)
            match_events = [(t, e) for t, e in events if any(s <= t <= ee for s, ee in enabled_periods)]
            if match_events:
                print(f"  {client_name}: {len(match_events)} state changes during match")
                for t, evt in match_events[:5]:
                    print(f"    t={t:.2f}s: {evt}")
            else:
                connected = sum(1 for _, v in series if v)
                status = "connected" if connected > len(series) / 2 else "disconnected"
                print(f"  {client_name}: stable ({status})")

    print()


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../logs"

    if os.path.isfile(log_dir):
        analyze_file(log_dir)
        return

    files = sorted([f for f in os.listdir(log_dir) if f.endswith(".wpilog") and "_sim" not in f])

    if not files:
        print("No .wpilog files found!")
        return

    print(f"Found {len(files)} log files in {log_dir}")
    for f in files:
        try:
            analyze_file(os.path.join(log_dir, f))
        except Exception as e:
            import traceback
            print(f"  ERROR analyzing {f}: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()
