"""NT4 client wrapper for live FRC robot telemetry via NetworkTables."""

import ntcore
import time

# Module-level singleton state
_inst: ntcore.NetworkTableInstance | None = None
_connected: bool = False
_server_addr: str = ""
_team: int = 41

# Keys that the robot allows writing to
WRITABLE_KEYS = {
    "/Vision/enableMultiTag": bool,
    "/Vision/enablePnpDistTrig": bool,
    "/AutoChooser/selected": str,
}

# Prefixes to try when resolving short user-facing keys
_RESOLVE_PREFIXES = [
    "",                          # raw key as given
    "/AdvantageKit/RealOutputs", # Logger.recordOutput paths
    "/AdvantageKit",             # Logger.processInputs paths
    "/RealOutputs",              # wpilog-style output paths
    "/SmartDashboard",           # SmartDashboard entries
]


def connect(target: str = "robot") -> str:
    """Connect to robot NT4 server.

    Args:
        target: "robot" (10.0.41.2), "sim" / "localhost" (127.0.0.1), or an IP address.

    Returns:
        Status message.
    """
    global _inst, _connected, _server_addr

    if _connected and _inst is not None:
        disconnect()

    _inst = ntcore.NetworkTableInstance.getDefault()
    _inst.setServer

    if target in ("robot", "roborio"):
        _server_addr = f"10.0.{_team // 100}.{_team % 100}"
        _inst.setServerTeam(_team)
    elif target in ("sim", "localhost", "127.0.0.1"):
        _server_addr = "127.0.0.1"
        _inst.setServer(_server_addr)
    else:
        _server_addr = target
        _inst.setServer(_server_addr)

    _inst.startClient4("frc41-mcp")
    _connected = True

    # Wait briefly for connection
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if _inst.isConnected():
            return f"Connected to NT4 at {_server_addr}"
        time.sleep(0.1)

    return f"NT4 client started for {_server_addr} (not yet connected — robot may be off)"


def disconnect() -> str:
    """Disconnect from NT4."""
    global _inst, _connected, _server_addr

    if _inst is not None:
        _inst.stopClient()
        ntcore.NetworkTableInstance.destroy(_inst)
        _inst = None
    _connected = False
    _server_addr = ""
    return "Disconnected from NT4"


def status() -> dict:
    """Return NT4 connection status."""
    if _inst is None or not _connected:
        return {"connected": False, "server": "", "peers": []}

    conns = _inst.getConnections()
    peers = []
    for c in conns:
        peers.append({
            "remote_id": c.remote_id,
            "remote_ip": c.remote_ip,
            "remote_port": c.remote_port,
            "protocol_version": c.protocol_version,
        })

    return {
        "connected": _inst.isConnected(),
        "server": _server_addr,
        "peers": peers,
    }


def _require_connected():
    """Raise if not connected."""
    if _inst is None or not _connected:
        raise RuntimeError("Not connected to NT4. Use nt_connect first.")


def _resolve_key(short_key: str) -> str | None:
    """Resolve a user-facing short key to the actual NT topic name.

    Tries the key with various prefixes until a topic with data is found.
    """
    _require_connected()

    # Normalize: ensure key starts with /
    if not short_key.startswith("/"):
        short_key = "/" + short_key

    topics = _inst.getTopics()
    topic_names = {t.getName() for t in topics}

    for prefix in _RESOLVE_PREFIXES:
        candidate = prefix + short_key
        if candidate in topic_names:
            return candidate

    # Try without leading slash on the key after prefix
    for prefix in _RESOLVE_PREFIXES:
        if prefix:
            candidate = prefix + "/" + short_key.lstrip("/")
            if candidate in topic_names:
                return candidate

    return None


def _get_topic_value(topic_name: str):
    """Read the current value of a topic by name."""
    _require_connected()

    topics = _inst.getTopics()
    for t in topics:
        if t.getName() == topic_name:
            type_str = t.getTypeString()
            sub = t.genericSubscribe()
            val = sub.get()
            sub.close()
            return val.value() if val is not None else None

    return None


def get(key: str) -> dict:
    """Get a single NT value by path.

    Args:
        key: NT path, e.g. "/Odom/xMeters" or "Swerve/NW/driveErrorMetersPerSec"

    Returns:
        Dict with key, value, resolved_path.
    """
    _require_connected()

    resolved = _resolve_key(key)
    if resolved is None:
        return {"error": f"Key '{key}' not found in NT4", "hint": "Use nt_list to see available keys"}

    value = _get_topic_value(resolved)
    return {"key": key, "resolved_path": resolved, "value": value}


def get_subtable(prefix: str) -> dict:
    """Get all values under a prefix.

    Args:
        prefix: NT path prefix, e.g. "/Shooter" or "/Swerve/NW"

    Returns:
        Dict of {short_key: value}.
    """
    _require_connected()

    if not prefix.startswith("/"):
        prefix = "/" + prefix

    topics = _inst.getTopics()
    results = {}

    for t in topics:
        name = t.getName()
        # Check if topic matches under any resolved prefix
        matched_short = None
        for pfx in _RESOLVE_PREFIXES:
            full_prefix = pfx + prefix
            if name.startswith(full_prefix):
                # Strip to get short readable name
                matched_short = name[len(pfx):] if pfx else name
                break

        if matched_short is not None:
            sub = t.genericSubscribe()
            val = sub.get()
            sub.close()
            if val is not None:
                results[matched_short] = val.value()

    if not results:
        return {"error": f"No topics found under prefix '{prefix}'", "hint": "Use nt_list to see available keys"}

    return results


def list_topics(prefix: str = "") -> list[str]:
    """List all available NT topic names, optionally filtered by prefix.

    Returns short-form keys (with /AdvantageKit/RealOutputs/ etc. stripped).
    """
    _require_connected()

    topics = _inst.getTopics()
    names = []

    for t in topics:
        name = t.getName()
        # Strip common prefixes for readability
        short = name
        for pfx in ["/AdvantageKit/RealOutputs", "/AdvantageKit", "/RealOutputs", "/SmartDashboard"]:
            if name.startswith(pfx + "/"):
                short = name[len(pfx):]
                break

        if prefix:
            norm_prefix = prefix if prefix.startswith("/") else "/" + prefix
            if short.startswith(norm_prefix) or name.startswith(norm_prefix):
                names.append(short)
        else:
            names.append(short)

    return sorted(set(names))


def set_value(key: str, value) -> str:
    """Set a writable NT value.

    Only allows writing to known-writable keys (vision toggles, auto selection).
    """
    _require_connected()

    # Normalize key
    if not key.startswith("/"):
        key = "/" + key

    if key not in WRITABLE_KEYS:
        allowed = ", ".join(WRITABLE_KEYS.keys())
        return f"Error: '{key}' is not writable. Writable keys: {allowed}"

    expected_type = WRITABLE_KEYS[key]

    # Parse value to correct type
    if expected_type == bool:
        if isinstance(value, str):
            value = value.lower() in ("true", "1", "yes")
        pub = _inst.getBooleanTopic(key).publish()
        pub.set(bool(value))
    elif expected_type == str:
        pub = _inst.getStringTopic(key).publish()
        pub.set(str(value))
    elif expected_type in (int, float):
        pub = _inst.getDoubleTopic(key).publish()
        pub.set(float(value))

    return f"Set {key} = {value}"


def get_auto_options() -> dict:
    """Get available autonomous routines and current selection."""
    _require_connected()

    options_topic = None
    selected_topic = None
    active_topic = None

    for t in _inst.getTopics():
        name = t.getName()
        if name.endswith("/AutoChooser/options") or name == "/AutoChooser/options":
            options_topic = t
        elif name.endswith("/AutoChooser/selected") or name == "/AutoChooser/selected":
            selected_topic = t
        elif name.endswith("/AutoChooser/active") or name == "/AutoChooser/active":
            active_topic = t

    result = {}

    if options_topic:
        sub = options_topic.genericSubscribe()
        val = sub.get()
        sub.close()
        result["options"] = val.value() if val else []

    if selected_topic:
        sub = selected_topic.genericSubscribe()
        val = sub.get()
        sub.close()
        result["selected"] = val.value() if val else ""

    if active_topic:
        sub = active_topic.genericSubscribe()
        val = sub.get()
        sub.close()
        result["active"] = val.value() if val else ""

    return result if result else {"error": "AutoChooser topics not found"}
