"""Java code navigation, analysis, and scaffolding tools for FRC robot code."""

import os
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
SRC_ROOT = PROJECT_ROOT / "src" / "main" / "java" / "frc" / "robot"
SUBSYSTEM_DIR = SRC_ROOT / "subsystem"
COMMANDS_DIR = SRC_ROOT / "commands"


def search_code(query: str, file_glob: str = "*.java", max_results: int = 50) -> list[dict]:
    """Regex search across robot Java source files.

    Args:
        query: Regex pattern to search for.
        file_glob: Glob pattern for files to search (default: *.java).
        max_results: Maximum number of results.

    Returns:
        List of {file, line, text} dicts.
    """
    results = []
    pattern = re.compile(query, re.IGNORECASE)
    search_dir = SRC_ROOT

    for java_file in search_dir.rglob(file_glob):
        try:
            text = java_file.read_text(encoding="utf-8")
        except Exception:
            continue

        for i, line in enumerate(text.splitlines(), 1):
            if pattern.search(line):
                rel_path = java_file.relative_to(PROJECT_ROOT)
                results.append({
                    "file": str(rel_path).replace("\\", "/"),
                    "line": i,
                    "text": line.strip(),
                })
                if len(results) >= max_results:
                    return results

    return results


def read_file(path: str, start_line: int = 1, end_line: int | None = None) -> dict:
    """Read a source file from the project.

    Args:
        path: Path relative to project root (e.g., "src/main/java/frc/robot/Robot.java").
        start_line: First line to include (1-based).
        end_line: Last line to include. None = read to end.

    Returns:
        {path, content, total_lines}.
    """
    full_path = PROJECT_ROOT / path.replace("/", os.sep)
    if not full_path.exists():
        return {"error": f"File not found: {path}"}

    try:
        text = full_path.read_text(encoding="utf-8")
    except Exception as e:
        return {"error": f"Failed to read {path}: {e}"}

    lines = text.splitlines()
    total = len(lines)

    start_idx = max(0, start_line - 1)
    end_idx = end_line if end_line else total

    selected = lines[start_idx:end_idx]
    numbered = "\n".join(f"{start_line + i:4d}  {line}" for i, line in enumerate(selected))

    return {
        "path": path,
        "content": numbered,
        "total_lines": total,
        "showing": f"{start_line}-{min(end_idx, total)}",
    }


def list_subsystems() -> list[dict]:
    """List all subsystems with their HW class, Inputs class, CAN IDs, and NT log paths.

    Scans src/main/java/frc/robot/subsystem/ directories.
    """
    if not SUBSYSTEM_DIR.exists():
        return []

    subsystems = []

    for sub_dir in sorted(SUBSYSTEM_DIR.iterdir()):
        if not sub_dir.is_dir():
            continue

        name = sub_dir.name
        info = {
            "name": name,
            "path": str(sub_dir.relative_to(PROJECT_ROOT)).replace("\\", "/"),
            "files": [],
            "hw_class": None,
            "inputs_class": None,
            "can_ids": [],
            "nt_paths": [],
            "motor_types": [],
        }

        for java_file in sorted(sub_dir.glob("*.java")):
            rel = str(java_file.relative_to(PROJECT_ROOT)).replace("\\", "/")
            info["files"].append(rel)

            try:
                text = java_file.read_text(encoding="utf-8")
            except Exception:
                continue

            # Detect HW class
            if "HW" in java_file.stem and "class" in text:
                info["hw_class"] = java_file.stem

            # Detect Inputs class
            if "@AutoLog" in text and "Inputs" in java_file.stem:
                info["inputs_class"] = java_file.stem

            # Extract CAN IDs
            for m in re.finditer(r'new\s+(TalonFX|CANcoder|SparkMax|SparkFlex|Pigeon2)\s*\(\s*(\d+)', text):
                device_type, can_id = m.group(1), int(m.group(2))
                info["can_ids"].append({"device": device_type, "id": can_id})

            # Extract motor types
            for motor_type in ["TalonFX", "SparkMax", "SparkFlex", "TalonSRX"]:
                if motor_type in text and motor_type not in info["motor_types"]:
                    info["motor_types"].append(motor_type)

            # Extract NT log paths
            for m in re.finditer(r'Logger\.(?:recordOutput|processInputs)\s*\(\s*"([^"]+)"', text):
                path = m.group(1)
                if path not in info["nt_paths"]:
                    info["nt_paths"].append(path)

        subsystems.append(info)

    return subsystems


def list_commands() -> list[dict]:
    """List all commands grouped by subsystem.

    Scans src/main/java/frc/robot/commands/ directories.
    """
    if not COMMANDS_DIR.exists():
        return []

    commands = []

    for item in sorted(COMMANDS_DIR.rglob("*.java")):
        try:
            text = item.read_text(encoding="utf-8")
        except Exception:
            continue

        rel = str(item.relative_to(PROJECT_ROOT)).replace("\\", "/")
        class_name = item.stem

        # Determine command type
        cmd_type = "unknown"
        if "extends Command" in text:
            cmd_type = "command"
        elif "InstantCommand" in text:
            cmd_type = "instant"
        elif "RunCommand" in text:
            cmd_type = "run"
        elif "SequentialCommandGroup" in text:
            cmd_type = "sequence"
        elif "ParallelCommandGroup" in text:
            cmd_type = "parallel"

        # Determine subsystem group
        parent = item.parent.name
        if parent == "commands":
            parent = "global"

        # Extract addRequirements
        requirements = re.findall(r'addRequirements\s*\(\s*(\w+)', text)

        commands.append({
            "name": class_name,
            "file": rel,
            "group": parent,
            "type": cmd_type,
            "requirements": requirements,
        })

    return commands


def analyze_subsystem(subsystem_name: str) -> dict:
    """Deep analysis of a specific subsystem.

    Args:
        subsystem_name: Subsystem directory name (e.g., "drive", "shooter", "intake").

    Returns:
        Detailed analysis including CAN config, gains, control modes, NT paths,
        and commands that reference this subsystem.
    """
    sub_dir = SUBSYSTEM_DIR / subsystem_name
    if not sub_dir.exists():
        available = [d.name for d in SUBSYSTEM_DIR.iterdir() if d.is_dir()]
        return {"error": f"Subsystem '{subsystem_name}' not found. Available: {available}"}

    analysis = {
        "name": subsystem_name,
        "files": {},
        "can_devices": [],
        "gains": [],
        "constants": [],
        "nt_paths": [],
        "control_modes": [],
        "commands_using": [],
    }

    # Analyze subsystem source files
    for java_file in sorted(sub_dir.glob("*.java")):
        rel = str(java_file.relative_to(PROJECT_ROOT)).replace("\\", "/")
        try:
            text = java_file.read_text(encoding="utf-8")
        except Exception:
            continue

        lines = text.splitlines()
        analysis["files"][java_file.stem] = {
            "path": rel,
            "lines": len(lines),
        }

        # CAN devices with full context
        for m in re.finditer(r'new\s+(TalonFX|CANcoder|SparkMax|SparkFlex|Pigeon2)\s*\(\s*(\d+)', text):
            analysis["can_devices"].append({
                "type": m.group(1),
                "id": int(m.group(2)),
                "file": java_file.stem,
            })

        # PID gains and feedforward constants
        for m in re.finditer(r'((?:static\s+)?(?:final\s+)?double\s+\w*k[PIDS]\w*)\s*=\s*([^;]+)', text):
            analysis["gains"].append({
                "declaration": m.group(1).strip(),
                "value": m.group(2).strip(),
                "file": java_file.stem,
            })
        for m in re.finditer(r'((?:static\s+)?(?:final\s+)?double\s+\w*(?:MAX|MIN|RATIO|ZERO|OFFSET)\w*)\s*=\s*([^;]+)', text):
            analysis["constants"].append({
                "declaration": m.group(1).strip(),
                "value": m.group(2).strip(),
                "file": java_file.stem,
            })

        # NT log paths
        for m in re.finditer(r'Logger\.(?:recordOutput|processInputs)\s*\(\s*"([^"]+)"', text):
            if m.group(1) not in analysis["nt_paths"]:
                analysis["nt_paths"].append(m.group(1))

        # Control modes
        for mode in ["VelocityVoltage", "PositionVoltage", "MotionMagicExpo", "VoltageOut",
                      "DutyCycleOut", "TorqueCurrentFOC", "setVoltage"]:
            if mode in text and mode not in analysis["control_modes"]:
                analysis["control_modes"].append(mode)

    # Find commands that reference this subsystem
    if COMMANDS_DIR.exists():
        for cmd_file in COMMANDS_DIR.rglob("*.java"):
            try:
                text = cmd_file.read_text(encoding="utf-8")
            except Exception:
                continue

            if subsystem_name in text.lower() or f"import.*{subsystem_name}" in text.lower():
                rel = str(cmd_file.relative_to(PROJECT_ROOT)).replace("\\", "/")
                analysis["commands_using"].append({
                    "name": cmd_file.stem,
                    "file": rel,
                })

    return analysis


def check_can_ids() -> dict:
    """Scan all HW classes for CAN device IDs and detect conflicts.

    Returns:
        {devices: [{type, id, bus, file, subsystem}], conflicts: [...]}
    """
    devices = []

    # Scan all Java files in subsystem directory
    for java_file in SUBSYSTEM_DIR.rglob("*.java"):
        try:
            text = java_file.read_text(encoding="utf-8")
        except Exception:
            continue

        subsystem = java_file.parent.name
        rel = str(java_file.relative_to(PROJECT_ROOT)).replace("\\", "/")

        # Detect bus usage
        bus = "rio"  # default
        if "driveBus" in text or '"Ducky"' in text:
            bus = "Ducky"

        for m in re.finditer(r'new\s+(TalonFX|CANcoder|SparkMax|SparkFlex|Pigeon2)\s*\(\s*(\d+)', text):
            device_type = m.group(1)
            can_id = int(m.group(2))

            # SparkMax/SparkFlex are always on rio bus
            if device_type in ("SparkMax", "SparkFlex"):
                dev_bus = "rio"
            else:
                dev_bus = bus

            devices.append({
                "type": device_type,
                "id": can_id,
                "bus": dev_bus,
                "file": rel,
                "subsystem": subsystem,
            })

    # Also check RobotContainer for IMU, etc.
    rc_file = SRC_ROOT / "RobotContainer.java"
    if rc_file.exists():
        try:
            text = rc_file.read_text(encoding="utf-8")
            for m in re.finditer(r'new\s+(TalonFX|CANcoder|SparkMax|SparkFlex|Pigeon2)\s*\(\s*(\d+)', text):
                devices.append({
                    "type": m.group(1),
                    "id": int(m.group(2)),
                    "bus": "Ducky" if "driveBus" in text else "rio",
                    "file": "src/main/java/frc/robot/RobotContainer.java",
                    "subsystem": "robot",
                })
        except Exception:
            pass

    # Detect conflicts (same ID on same bus)
    conflicts = []
    by_bus = {}
    for d in devices:
        key = (d["bus"], d["id"])
        by_bus.setdefault(key, []).append(d)

    for key, devs in by_bus.items():
        if len(devs) > 1:
            conflicts.append({
                "bus": key[0],
                "id": key[1],
                "devices": devs,
            })

    return {
        "devices": sorted(devices, key=lambda d: (d["bus"], d["id"])),
        "conflicts": conflicts,
        "total": len(devices),
    }


# ── Code Generation Templates ──────────────────────────────────────────────

def generate_command(name: str, subsystem: str, command_type: str = "command",
                     description: str = "") -> dict:
    """Generate a new WPILib command file.

    Args:
        name: Command class name (e.g., "ArmExtend").
        subsystem: Subsystem name for package (e.g., "arm") and addRequirements.
        command_type: "command" (full), "instant", or "run".
        description: Optional description comment.

    Returns:
        {path, content} of the generated file.
    """
    package = f"frc.robot.commands.{subsystem}"
    package_dir = COMMANDS_DIR / subsystem
    file_path = package_dir / f"{name}.java"
    rel_path = str(file_path.relative_to(PROJECT_ROOT)).replace("\\", "/")

    desc_comment = f"/** {description} */\n" if description else ""

    if command_type == "instant":
        content = f"""package {package};

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

{desc_comment}public class {name} extends InstantCommand {{
\tpublic {name}() {{
\t\tsuper(() -> {{
\t\t\t// TODO: implement
\t\t}}, {subsystem});
\t}}
}}
"""
    elif command_type == "run":
        content = f"""package {package};

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.RunCommand;

{desc_comment}public class {name} extends RunCommand {{
\tpublic {name}() {{
\t\tsuper(() -> {{
\t\t\t// TODO: implement execute
\t\t}}, {subsystem});
\t}}
}}
"""
    else:
        content = f"""package {package};

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

{desc_comment}public class {name} extends Command {{
\tpublic {name}() {{
\t\taddRequirements({subsystem});
\t}}

\t@Override
\tpublic void initialize() {{
\t}}

\t@Override
\tpublic void execute() {{
\t\t// TODO: implement
\t}}

\t@Override
\tpublic void end(boolean interrupted) {{
\t}}

\t@Override
\tpublic boolean isFinished() {{
\t\treturn false;
\t}}
}}
"""

    return {
        "path": rel_path,
        "content": content,
        "wrote": False,
        "message": f"Generated {command_type} command '{name}' for subsystem '{subsystem}'",
    }


def generate_subsystem(name: str, motor_type: str = "TalonFX",
                       motor_count: int = 1, description: str = "") -> dict:
    """Generate a new HW subsystem following the project's init/sense/actuate pattern.

    Args:
        name: Subsystem name (e.g., "Arm"). Used for class names and package.
        motor_type: "TalonFX", "SparkMax", or "SparkFlex".
        motor_count: Number of motors.
        description: Optional description.

    Returns:
        Dict with paths and content for the HW class and Inputs POJO.
    """
    package_name = name.lower()
    package = f"frc.robot.subsystem.{package_name}"
    sub_dir = SUBSYSTEM_DIR / package_name

    hw_name = f"{name}HW"
    inputs_name = f"{name}Inputs"
    log_path = f"/{name}"

    desc_comment = f"/** {description} */\n" if description else ""

    # Generate Inputs POJO
    inputs_content = f"""package {package};

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class {inputs_name} {{
\tpublic double posRadians;
\tpublic double velRadiansPerSec;
\tpublic double voltageVolts;
\tpublic double currentAmps;
}}
"""

    # Generate HW class
    if motor_type == "TalonFX":
        motor_imports = """import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;"""
        motor_fields = "\n".join(
            f"\tpublic TalonFX motor{i};" for i in range(motor_count)
        )
        motor_init = "\n".join(
            f"\t\tmotor{i} = new TalonFX({i + 1}); // TODO: set CAN ID" for i in range(motor_count)
        )
        motor_sense = f"""\t\tinputs.posRadians = motor0.getPosition().getValueAsDouble() * 2.0 * PI;
\t\tinputs.velRadiansPerSec = motor0.getVelocity().getValueAsDouble() * 2.0 * PI;
\t\tinputs.voltageVolts = motor0.getMotorVoltage().getValueAsDouble();
\t\tinputs.currentAmps = motor0.getStatorCurrent().getValueAsDouble();"""
        motor_actuate = "\t\tmotor0.setControl(new VoltageOut(voltage));"
    elif motor_type in ("SparkMax", "SparkFlex"):
        spark_class = motor_type
        motor_imports = f"""import com.revrobotics.spark.{spark_class};
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.{spark_class}Config;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;"""
        motor_fields = "\n".join(
            f"\tpublic {spark_class} motor{i};\n\tpublic RelativeEncoder encoder{i};"
            for i in range(motor_count)
        )
        motor_init = "\n".join(
            f"""\t\tmotor{i} = new {spark_class}({i + 1}, MotorType.kBrushless); // TODO: set CAN ID
\t\t{spark_class}Config config{i} = new {spark_class}Config();
\t\tconfig{i}.smartCurrentLimit(40);
\t\tconfig{i}.idleMode(IdleMode.kBrake);
\t\tmotor{i}.configure(config{i}, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
\t\tencoder{i} = motor{i}.getEncoder();"""
            for i in range(motor_count)
        )
        motor_sense = f"""\t\tinputs.posRadians = encoder0.getPosition();
\t\tinputs.velRadiansPerSec = encoder0.getVelocity();
\t\tinputs.voltageVolts = motor0.getBusVoltage() * motor0.getAppliedOutput();
\t\tinputs.currentAmps = motor0.getOutputCurrent();"""
        motor_actuate = "\t\tmotor0.setVoltage(voltage);"
    else:
        return {"error": f"Unknown motor type: {motor_type}. Use TalonFX, SparkMax, or SparkFlex."}

    hw_content = f"""package {package};

import static java.lang.Math.*;

{motor_imports}
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

{desc_comment}public class {hw_name} {{
{motor_fields}

\tpublic void init() {{
\t\tif (!Robot.isReal()) return;

{motor_init}
\t}}

\tpublic void sense({inputs_name} inputs) {{
\t\tif (!Robot.isReal()) return;

{motor_sense}
\t}}

\tpublic void actuate(double voltage) {{
\t\tLogger.recordOutput("{log_path}/targetVoltage", voltage);

\t\tif (!Robot.isReal()) return;

{motor_actuate}
\t}}
}}
"""

    hw_path = str((sub_dir / f"{hw_name}.java").relative_to(PROJECT_ROOT)).replace("\\", "/")
    inputs_path = str((sub_dir / f"{inputs_name}.java").relative_to(PROJECT_ROOT)).replace("\\", "/")

    return {
        "files": [
            {"path": hw_path, "content": hw_content},
            {"path": inputs_path, "content": inputs_content},
        ],
        "wrote": False,
        "message": f"Generated {motor_type}-based subsystem '{name}' with {motor_count} motor(s)",
        "next_steps": [
            f"Add static {name} instance to RobotContainer",
            f"Call {package_name}.init() in RobotContainer.init()",
            f"Add {package_name}.sense() and {package_name}.actuate() to RobotContainer.periodic()",
            "Set correct CAN IDs in the HW class",
        ],
    }
