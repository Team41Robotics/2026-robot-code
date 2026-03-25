# FRC Team 41 MCP Server

Live robot debugging + Java code tools for Claude Code.

## Setup

```bash
cd mcp
pip install -r requirements.txt
```

## Claude Code Integration

The `.mcp.json` at the project root auto-registers the server. Restart Claude Code and verify:

```
/mcp
```

Should show `frc41: connected` with 26 tools.

### Manual Registration

```bash
claude mcp add --scope user frc41 -- python mcp/server.py
```

## Tools (26)

### Live Robot (NT4)
| Tool | What it does |
|------|-------------|
| `nt_connect` | Connect to robot (`robot`/`sim`/IP) |
| `nt_disconnect` | Disconnect |
| `nt_status` | Connection status |
| `nt_get` | Read a single NT value |
| `nt_get_subtable` | Read all values under a prefix |
| `nt_list` | List all NT topics |
| `nt_set` | Set writable NT value (vision toggles, auto select) |
| `select_auto` | Pick autonomous routine |

### Diagnostics (live)
| Tool | What it does |
|------|-------------|
| `robot_health` | GC, timing, match period, warnings |
| `drive_status` | Swerve pose, speeds, per-module errors |
| `shooter_status` | Turret/hood/flywheel status + onTarget |
| `vision_status` | Camera tag counts, pose estimates |
| `subsystem_timing` | Per-subsystem loop times |
| `match_status` | Match period + time remaining |

### Log Analysis (offline)
| Tool | What it does |
|------|-------------|
| `list_logs` | List .wpilog files |
| `log_keys` | List signals in a log |
| `log_query` | Extract time series data |
| `log_summary` | Statistics for signals |

### Java Code
| Tool | What it does |
|------|-------------|
| `search_code` | Regex search across source |
| `read_file` | Read a source file |
| `list_subsystems` | All subsystems with CAN IDs, motors, NT paths |
| `list_commands` | All commands grouped by subsystem |
| `generate_command` | Scaffold a new command |
| `generate_subsystem` | Scaffold HW class + Inputs POJO |
| `analyze_subsystem` | Deep dive on one subsystem |
| `check_can_ids` | Full CAN bus map + conflict detection |

## Usage Examples

```
# Connect to live robot
> nt_connect robot

# Check health
> robot_health

# Watch swerve errors
> drive_status

# Analyze a log file
> log_summary akit_26-03-07_23-51-56_njwas_q38.wpilog

# Find all PID gains
> search_code "kP|kD|kI|kV|kS"

 # Scan CAN bus
> check_can_ids
```

## Architecture

```
server.py       ─── MCP tool definitions + dispatch
nt_client.py    ─── NT4 singleton (pyntcore)
log_reader.py   ─── .wpilog reader (wpiutil)
java_tools.py   ─── Code search, analysis, scaffolding
```

NT4 key resolution: the server transparently maps short paths like `/Shooter/onTarget` to the actual AdvantageKit NT path (`/AdvantageKit/RealOutputs/Shooter/onTarget` or `/RealOutputs/Shooter/onTarget`).
