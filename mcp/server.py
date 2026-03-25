"""FRC Team 41 MCP Server — Live robot debugging + Java code tools.

30 tools for NT4 telemetry, diagnostics, build/deploy, log analysis,
code navigation, and subsystem/command scaffolding.

Usage:
    python server.py
"""

import asyncio
import json
import os
import subprocess
import sys
from pathlib import Path

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

# Local modules
sys.path.insert(0, str(Path(__file__).parent))
import nt_client
import log_reader
import java_tools

PROJECT_ROOT = Path(__file__).parent.parent
JAVA_HOME = "C:/Users/Public/wpilib/2026/jdk"

app = Server("frc41-mcp")


# ── Helpers ─────────────────────────────────────────────────────────────────

def _text(data) -> list[TextContent]:
    """Convert data to MCP TextContent response."""
    if isinstance(data, (dict, list)):
        return [TextContent(type="text", text=json.dumps(data, indent=2, default=str))]
    return [TextContent(type="text", text=str(data))]


def _run_gradle(task: str, timeout: int = 180) -> str:
    """Run a Gradle task with WPILib JDK."""
    env = os.environ.copy()
    env["JAVA_HOME"] = JAVA_HOME
    env["PATH"] = f"{JAVA_HOME}/bin;{env.get('PATH', '')}"

    gradlew = str(PROJECT_ROOT / "gradlew.bat") if os.name == "nt" else "./gradlew"
    cmd = [gradlew, task] if os.name != "nt" else ["cmd", "/c", gradlew, task]

    try:
        result = subprocess.run(
            cmd, cwd=str(PROJECT_ROOT),
            capture_output=True, text=True, timeout=timeout, env=env,
        )
        output = result.stdout
        if result.stderr:
            output += "\n--- STDERR ---\n" + result.stderr
        return f"Exit code: {result.returncode}\n{output}"
    except subprocess.TimeoutExpired:
        return f"Error: Gradle task '{task}' timed out after {timeout}s"
    except Exception as e:
        return f"Error running gradle {task}: {e}"


# ── Tool Registration ──────────────────────────────────────────────────────

@app.list_tools()
async def list_tools() -> list[Tool]:
    return [
        # Connection (3)
        Tool(name="nt_connect", description="Connect to robot NT4 server. target: 'robot' (10.0.41.2), 'sim' (localhost), or IP address.",
             inputSchema={"type": "object", "properties": {"target": {"type": "string", "default": "robot", "description": "robot, sim, or IP"}}}),
        Tool(name="nt_disconnect", description="Disconnect from NT4 server.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="nt_status", description="Show NT4 connection status, server address, and connected peers.",
             inputSchema={"type": "object", "properties": {}}),

        # Live Data (3)
        Tool(name="nt_get", description="Get a single NetworkTables value by path. Examples: /Odom/xMeters, /Shooter/onTarget, Swerve/NW/driveErrorMetersPerSec",
             inputSchema={"type": "object", "properties": {"key": {"type": "string", "description": "NT path"}}, "required": ["key"]}),
        Tool(name="nt_get_subtable", description="Get all NT values under a prefix. Examples: /Shooter, /Swerve/NW, /Vision/DuckyNE",
             inputSchema={"type": "object", "properties": {"prefix": {"type": "string", "description": "NT path prefix"}}, "required": ["prefix"]}),
        Tool(name="nt_list", description="List all available NT topic names, optionally filtered by prefix.",
             inputSchema={"type": "object", "properties": {"prefix": {"type": "string", "default": "", "description": "Filter prefix"}}}),

        # Live Control (2)
        Tool(name="nt_set", description="Set a writable NT value. Allowed: /Vision/enableMultiTag (bool), /Vision/enablePnpDistTrig (bool), /AutoChooser/selected (string).",
             inputSchema={"type": "object", "properties": {"key": {"type": "string"}, "value": {"type": "string"}}, "required": ["key", "value"]}),
        Tool(name="select_auto", description="Select autonomous routine. Pass empty name to list available options.",
             inputSchema={"type": "object", "properties": {"name": {"type": "string", "default": "", "description": "Auto routine name, or empty to list"}}}),

        # Diagnostics (6)
        Tool(name="robot_health", description="Overall robot health: GC stats, loop timing, match period. Flags warnings for high heap usage or slow loops.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="drive_status", description="Swerve drive status: pose (x/y/rot), speeds, per-module drive/turn errors. Flags modules with large errors.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="shooter_status", description="Shooter status: turret/hood/flywheel positions, targets, errors, onTarget flag.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="vision_status", description="Vision pipeline status: per-camera tag count, tag IDs, pose estimates, enabled flags.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="subsystem_timing", description="Loop timing breakdown: sense/actuate time per subsystem and CommandScheduler. Flags any >5ms.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="match_status", description="Match state: period (AUTO/TELEOP/etc), time remaining, alliance hub status.",
             inputSchema={"type": "object", "properties": {}}),

        # Build & Deploy (4)
        Tool(name="gradle_build", description="Run 'gradlew build' with WPILib JDK. Returns stdout/stderr.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="gradle_deploy", description="Run 'gradlew deploy' to deploy code to RoboRIO.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="gradle_task", description="Run an arbitrary Gradle task (e.g., 'simulateJava', 'test', 'spotlessCheck').",
             inputSchema={"type": "object", "properties": {"task": {"type": "string", "description": "Gradle task name"}}, "required": ["task"]}),
        Tool(name="format_code", description="Run 'gradlew spotlessApply' to auto-format all code.",
             inputSchema={"type": "object", "properties": {}}),

        # Log Analysis (4)
        Tool(name="list_logs", description="List .wpilog files in logs/ directory with size and date.",
             inputSchema={"type": "object", "properties": {"log_dir": {"type": "string", "description": "Optional log directory path"}}}),
        Tool(name="log_keys", description="List all signal keys in a .wpilog file.",
             inputSchema={"type": "object", "properties": {"log_file": {"type": "string", "description": "Log filename or path"}}, "required": ["log_file"]}),
        Tool(name="log_query", description="Extract time series data from a .wpilog file for specific keys over a time range.",
             inputSchema={"type": "object", "properties": {
                 "log_file": {"type": "string"}, "keys": {"type": "array", "items": {"type": "string"}},
                 "start_sec": {"type": "number", "description": "Start time in seconds from log start"},
                 "end_sec": {"type": "number", "description": "End time in seconds"},
             }, "required": ["log_file", "keys"]}),
        Tool(name="log_summary", description="Statistical summary (min/max/mean/std) of numeric signals in a .wpilog file.",
             inputSchema={"type": "object", "properties": {
                 "log_file": {"type": "string"}, "keys": {"type": "array", "items": {"type": "string"}, "description": "Specific keys, or omit for all numeric"},
             }, "required": ["log_file"]}),

        # Java Code Navigation (4)
        Tool(name="search_code", description="Regex search across robot Java source files. Returns file, line number, and matched text.",
             inputSchema={"type": "object", "properties": {
                 "query": {"type": "string", "description": "Regex pattern"},
                 "file_glob": {"type": "string", "default": "*.java"},
             }, "required": ["query"]}),
        Tool(name="read_file", description="Read a source file from the project. Path relative to project root.",
             inputSchema={"type": "object", "properties": {
                 "path": {"type": "string", "description": "Relative path (e.g., src/main/java/frc/robot/Robot.java)"},
                 "start_line": {"type": "integer", "default": 1},
                 "end_line": {"type": "integer", "description": "Last line (omit for all)"},
             }, "required": ["path"]}),
        Tool(name="list_subsystems", description="List all subsystems with HW class, Inputs class, CAN IDs, NT log paths, and motor types.",
             inputSchema={"type": "object", "properties": {}}),
        Tool(name="list_commands", description="List all commands grouped by subsystem with command type (command/instant/run/sequence).",
             inputSchema={"type": "object", "properties": {}}),

        # Java Code Generation (4)
        Tool(name="generate_command", description="Scaffold a new WPILib command (command/instant/run type). Returns generated code without writing to disk.",
             inputSchema={"type": "object", "properties": {
                 "name": {"type": "string", "description": "Command class name"},
                 "subsystem": {"type": "string", "description": "Subsystem name (for package + addRequirements)"},
                 "command_type": {"type": "string", "enum": ["command", "instant", "run"], "default": "command"},
                 "description": {"type": "string", "default": ""},
             }, "required": ["name", "subsystem"]}),
        Tool(name="generate_subsystem", description="Scaffold a new HW subsystem with init/sense/actuate pattern and @AutoLog Inputs POJO.",
             inputSchema={"type": "object", "properties": {
                 "name": {"type": "string", "description": "Subsystem name (e.g., Arm)"},
                 "motor_type": {"type": "string", "enum": ["TalonFX", "SparkMax", "SparkFlex"], "default": "TalonFX"},
                 "motor_count": {"type": "integer", "default": 1},
                 "description": {"type": "string", "default": ""},
             }, "required": ["name"]}),
        Tool(name="analyze_subsystem", description="Deep analysis of a subsystem: CAN config, gains, control modes, NT paths, commands that use it.",
             inputSchema={"type": "object", "properties": {"name": {"type": "string", "description": "Subsystem directory name (drive, shooter, intake, etc.)"}}, "required": ["name"]}),
        Tool(name="check_can_ids", description="Scan all HW classes for CAN device IDs, show full bus map, and detect ID conflicts.",
             inputSchema={"type": "object", "properties": {}}),
    ]


# ── Tool Dispatch ──────────────────────────────────────────────────────────

@app.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    try:
        # ── Connection ──
        if name == "nt_connect":
            return _text(nt_client.connect(arguments.get("target", "robot")))
        elif name == "nt_disconnect":
            return _text(nt_client.disconnect())
        elif name == "nt_status":
            return _text(nt_client.status())

        # ── Live Data ──
        elif name == "nt_get":
            return _text(nt_client.get(arguments["key"]))
        elif name == "nt_get_subtable":
            return _text(nt_client.get_subtable(arguments["prefix"]))
        elif name == "nt_list":
            return _text(nt_client.list_topics(arguments.get("prefix", "")))

        # ── Live Control ──
        elif name == "nt_set":
            return _text(nt_client.set_value(arguments["key"], arguments["value"]))
        elif name == "select_auto":
            auto_name = arguments.get("name", "")
            if not auto_name:
                return _text(nt_client.get_auto_options())
            else:
                return _text(nt_client.set_value("/AutoChooser/selected", auto_name))

        # ── Diagnostics ──
        elif name == "robot_health":
            return _text(await _robot_health())
        elif name == "drive_status":
            return _text(await _drive_status())
        elif name == "shooter_status":
            return _text(await _shooter_status())
        elif name == "vision_status":
            return _text(await _vision_status())
        elif name == "subsystem_timing":
            return _text(await _subsystem_timing())
        elif name == "match_status":
            return _text(await _match_status())

        # ── Build & Deploy ──
        elif name == "gradle_build":
            return _text(await asyncio.to_thread(_run_gradle, "build"))
        elif name == "gradle_deploy":
            return _text(await asyncio.to_thread(_run_gradle, "deploy"))
        elif name == "gradle_task":
            return _text(await asyncio.to_thread(_run_gradle, arguments["task"]))
        elif name == "format_code":
            return _text(await asyncio.to_thread(_run_gradle, "spotlessApply"))

        # ── Log Analysis ──
        elif name == "list_logs":
            return _text(log_reader.list_logs(arguments.get("log_dir")))
        elif name == "log_keys":
            return _text(log_reader.get_keys(arguments["log_file"]))
        elif name == "log_query":
            return _text(log_reader.query(
                arguments["log_file"], arguments["keys"],
                arguments.get("start_sec"), arguments.get("end_sec"),
            ))
        elif name == "log_summary":
            return _text(log_reader.summary(
                arguments["log_file"], arguments.get("keys"),
            ))

        # ── Java Code Navigation ──
        elif name == "search_code":
            return _text(java_tools.search_code(
                arguments["query"], arguments.get("file_glob", "*.java"),
            ))
        elif name == "read_file":
            return _text(java_tools.read_file(
                arguments["path"], arguments.get("start_line", 1), arguments.get("end_line"),
            ))
        elif name == "list_subsystems":
            return _text(java_tools.list_subsystems())
        elif name == "list_commands":
            return _text(java_tools.list_commands())

        # ── Java Code Generation ──
        elif name == "generate_command":
            result = java_tools.generate_command(
                arguments["name"], arguments["subsystem"],
                arguments.get("command_type", "command"),
                arguments.get("description", ""),
            )
            return _text(result)
        elif name == "generate_subsystem":
            result = java_tools.generate_subsystem(
                arguments["name"],
                arguments.get("motor_type", "TalonFX"),
                arguments.get("motor_count", 1),
                arguments.get("description", ""),
            )
            return _text(result)
        elif name == "analyze_subsystem":
            return _text(java_tools.analyze_subsystem(arguments["name"]))
        elif name == "check_can_ids":
            return _text(java_tools.check_can_ids())

        else:
            return _text({"error": f"Unknown tool: {name}"})

    except Exception as e:
        return _text({"error": str(e)})


# ── Diagnostic Implementations ─────────────────────────────────────────────

async def _robot_health() -> dict:
    """Aggregated robot health check."""
    nt_client._require_connected()

    gc_data = nt_client.get_subtable("/GC")
    timing_data = nt_client.get_subtable("/Timing")

    warnings = []
    heap_pct = None
    if isinstance(gc_data, dict) and "error" not in gc_data:
        for k, v in gc_data.items():
            if "HeapUsedPercent" in k:
                heap_pct = v
                if isinstance(v, (int, float)) and v > 80:
                    warnings.append(f"HIGH HEAP USAGE: {v:.1f}%")

    if isinstance(timing_data, dict) and "error" not in timing_data:
        for k, v in timing_data.items():
            if isinstance(v, (int, float)) and v > 5.0:
                warnings.append(f"SLOW LOOP: {k} = {v:.1f}ms")

    match = nt_client.get("MatchPeriod")
    period = match.get("value", "UNKNOWN") if isinstance(match, dict) else "UNKNOWN"
    time_remaining = nt_client.get("PeriodTimeRemaining")
    remaining = time_remaining.get("value") if isinstance(time_remaining, dict) else None

    status = "OK"
    if warnings:
        status = "CRITICAL" if any("HIGH HEAP" in w for w in warnings) else "WARNING"

    return {
        "status": status,
        "match_period": period,
        "time_remaining": remaining,
        "heap_percent": heap_pct,
        "gc": gc_data if isinstance(gc_data, dict) and "error" not in gc_data else {},
        "timing": timing_data if isinstance(timing_data, dict) and "error" not in timing_data else {},
        "warnings": warnings,
    }


async def _drive_status() -> dict:
    """Swerve drive diagnostics."""
    nt_client._require_connected()

    odom = nt_client.get_subtable("/Odom")
    modules = {}
    warnings = []

    for mod in ["NW", "NE", "SW", "SE"]:
        mod_data = nt_client.get_subtable(f"/Swerve/{mod}")
        if isinstance(mod_data, dict) and "error" not in mod_data:
            modules[mod] = mod_data

            # Check for large errors
            for k, v in mod_data.items():
                if "turnError" in k and isinstance(v, (int, float)) and abs(v) > 0.1:
                    warnings.append(f"{mod} TURN ERROR: {v:.3f} rad ({abs(v)*57.3:.1f} deg)")
                if "driveError" in k and isinstance(v, (int, float)) and abs(v) > 0.5:
                    warnings.append(f"{mod} DRIVE ERROR: {v:.3f} m/s")

    speed = nt_client.get("/Swerve/realSpeedMetersPerSecond")

    return {
        "odometry": odom if isinstance(odom, dict) and "error" not in odom else {},
        "speed_mps": speed.get("value") if isinstance(speed, dict) else None,
        "modules": modules,
        "warnings": warnings,
    }


async def _shooter_status() -> dict:
    """Shooter diagnostics."""
    nt_client._require_connected()
    data = nt_client.get_subtable("/Shooter")
    if isinstance(data, dict) and "error" in data:
        return data

    on_target = None
    warnings = []
    for k, v in data.items():
        if "onTarget" in k:
            on_target = v
        if "turretError" in k and isinstance(v, (int, float)) and abs(v) > 0.05:
            warnings.append(f"TURRET ERROR: {v:.3f} rad ({abs(v)*57.3:.1f} deg)")
        if "hoodError" in k and isinstance(v, (int, float)) and abs(v) > 0.05:
            warnings.append(f"HOOD ERROR: {v:.3f} rad ({abs(v)*57.3:.1f} deg)")
        if "flywheelError" in k and isinstance(v, (int, float)) and abs(v) > 200:
            warnings.append(f"FLYWHEEL ERROR: {v:.0f} RPM")

    return {
        "on_target": on_target,
        "data": data,
        "warnings": warnings,
    }


async def _vision_status() -> dict:
    """Vision pipeline diagnostics."""
    nt_client._require_connected()

    cameras = {}
    for cam in ["DuckyNE", "KimmySE"]:
        cam_data = nt_client.get_subtable(f"/Vision/{cam}")
        if isinstance(cam_data, dict) and "error" not in cam_data:
            cameras[cam] = cam_data

    enabled = nt_client.get("/Vision/enabled")
    multi_tag = nt_client.get("/Vision/enableMultiTag")
    pnp = nt_client.get("/Vision/enablePnpDistTrig")

    return {
        "enabled": enabled.get("value") if isinstance(enabled, dict) else None,
        "enableMultiTag": multi_tag.get("value") if isinstance(multi_tag, dict) else None,
        "enablePnpDistTrig": pnp.get("value") if isinstance(pnp, dict) else None,
        "cameras": cameras,
    }


async def _subsystem_timing() -> dict:
    """Loop timing breakdown."""
    nt_client._require_connected()

    timing = nt_client.get_subtable("/Timing")
    if isinstance(timing, dict) and "error" in timing:
        return timing

    warnings = []
    for k, v in timing.items():
        if isinstance(v, (int, float)) and v > 5.0:
            warnings.append(f"{k}: {v:.1f}ms (>5ms threshold)")

    total = sum(v for v in timing.values() if isinstance(v, (int, float)))

    return {
        "timing_ms": timing,
        "total_ms": round(total, 2),
        "warnings": warnings,
    }


async def _match_status() -> dict:
    """Match state info."""
    nt_client._require_connected()

    period = nt_client.get("MatchPeriod")
    remaining = nt_client.get("PeriodTimeRemaining")
    hub = nt_client.get("AllianceHubStatus")
    red_won = nt_client.get("RedWonAuto")

    return {
        "period": period.get("value") if isinstance(period, dict) else None,
        "time_remaining": remaining.get("value") if isinstance(remaining, dict) else None,
        "alliance_hub_status": hub.get("value") if isinstance(hub, dict) else None,
        "red_won_auto": red_won.get("value") if isinstance(red_won, dict) else None,
    }


# ── Main ───────────────────────────────────────────────────────────────────

async def main():
    async with stdio_server() as (read_stream, write_stream):
        await app.run(read_stream, write_stream, app.create_initialization_options())


if __name__ == "__main__":
    asyncio.run(main())
