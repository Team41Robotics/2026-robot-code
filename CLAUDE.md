# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

WPILib-based FRC 2026 robot code (Java 17) built around a **sense → schedule → actuate** loop. Uses CTRE Phoenix 6 motors, PhotonVision, AdvantageKit (junction) logging, and swerve drive kinematics/estimation.

## Build commands

```bash
# Build
./gradlew build

# Format all Java/Gradle/XML/Markdown files (run before committing)
./gradlew spotlessApply

# Deploy to RoboRIO
./gradlew deploy

# Run simulation (opens WPILib sim GUI)
./gradlew simulateJava

# Log replay watcher
./gradlew replayWatch
```

On Windows use `gradlew.bat` instead of `./gradlew`.

## Architecture

### Main loop (`RobotContainer.periodic()`)

```
1. imu.sense()           // read sensors
2. drive.sense()
3. vision.sense()
4. CommandScheduler.run()  // execute commands
5. drive.actuate()       // write actuators
```

All subsystems are `static` fields in `RobotContainer` and statically imported (`import static frc.robot.RobotContainer.*`) in most files. Do not create new subsystem instances — reference the statics.

### Key files

| File | Purpose |
|------|---------|
| [Robot.java](src/main/java/frc/robot/Robot.java) | Lifecycle + Logger setup (real vs. simulation/replay) |
| [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) | Static subsystem wiring, `init()`/`periodic()` order, default commands |
| [FieldConstants.java](src/main/java/frc/robot/FieldConstants.java) | Field layout, AprilTag JSON lookup |
| [SwerveDrive.java](src/main/java/frc/robot/subsystem/drive/SwerveDrive.java) | Module configs, kinematics, pose estimator |
| [SwerveHW.java](src/main/java/frc/robot/subsystem/drive/SwerveHW.java) | TalonFX/CANcoder hardware interface pattern |
| [Vision.java](src/main/java/frc/robot/subsystem/vision/Vision.java) | PhotonPoseEstimator → drive pose estimator fusion |

### HW class pattern

Every hardware subsystem has three hooks:
- `init()` — configure hardware once at startup
- `sense(Inputs)` — read sensors, call `Logger.processInputs(path, inputs)`
- `actuate(...)` — write motor commands, call `Logger.recordOutput(path, value)`

`Inputs` is an `@AutoLog`-annotated POJO. See [SwerveInputs.java](src/main/java/frc/robot/subsystem/drive/SwerveInputs.java) for an example.

### Adding a new subsystem

1. Create a HW class with `init()`, `sense(Inputs)`, `actuate(...)` and an `@AutoLog` Inputs POJO.
2. Add static instance to `RobotContainer` and call `init()` there.
3. Add `sense()` before `CommandScheduler.run()` and `actuate()` after it in `RobotContainer.periodic()`.
4. Use `/SubsystemName/...` Logger paths consistent with existing subsystems.

## Naming conventions

**Measured / target / setpoint distinction:**
- Sensor readings: short names with no prefix — `angle`, `vel`, `drivePos`, `state`
- Commanded from higher-level code: `target*` — `targetVel`, `targetAng`
- Controller/trapezoid outputs: `setpoint*` — `setpointVel`, `setpointAng`

**Math-style locals in algorithms/commands:**
- `v_x`, `v_y` (field-relative), `v` or `mag` for magnitude, `theta` for heading, `w` for angular velocity

**Constants:**
- `ALL_CAPS_SUBSYSTEM_kGain` — e.g., `DRIVE_kS`, `DRIVE_kV`, `TURN_kP`, `TURN_kD`
- Physical limits: `MAX_VEL` (m/s), `MAX_W` (rad/s)

**Trapezoid profiles:**
- `TrapezoidProfile.Constraints` as `static final` constants
- `PIDController` and `TrapezoidProfile` as non-static instance fields (they hold state)

## Scope / Static / Final patterns

**Subsystem instances:**
- All hardware subsystems (e.g., `SwerveDrive`, `Vision`) are `static` fields in `RobotContainer` — initialized once at startup, accessible globally via static import.
- Do NOT create new instances of subsystems elsewhere in the codebase.

**Visibility (no encapsulation):**
- Subsystem and HW class fields: **`public`** by default (e.g., `SwerveDrive.pose`, `Vision.cameras`). Direct access is preferred over getters — it simplifies data flow and logging.
- Command classes: `public` everywhere

**Constants and configuration:**
- All constants: **`public static final`** — e.g., gains (`DRIVE_kS`), limits (`MAX_VEL`, `MAX_W`), config arrays (`SwerveDrive.configs`).
- Constants are typically defined in the HW class where they're used or in dedicated constant files (`FieldConstants`, `RobotContainer`).

**State holders (not final):**
- `PIDController`, `TrapezoidProfile`, pose estimators: **non-static instance fields**. These hold state across loop iterations and **must not be `final`** — they may be reset/reconfigured.
- Inputs POJOs: instance fields, auto-logged by AdvantageKit.

**Summary:**
- `static` = single instance, shared across code (subsystems, constants)
- `public` = default for fields (no encapsulation; direct access preferred)
- `final` = immutable reference for constants and configuration arrays
- Avoid `final` on stateful objects (controllers, estimators, profiles)

## Hardware config

- **CAN bus**: `RobotContainer.driveBus` (`CANBus("Ducky")`) — pass to all CTRE device constructors on drivetrain
- **Swerve module IDs + angle offsets**: `SwerveDrive.configs` array — update there, not scattered through code
- **AprilTag JSONs**: `src/main/deploy/apriltags/` — loaded by `FieldConstants.AprilTagLayoutType` at runtime

## Logging

Uses AdvantageKit (junction). Simulation runs in replay mode by default using `WPILOGReader` (see `Robot.robotInit()`). Log liberally — sensor readings, setpoints, and control outputs — to enable post-match replay and debugging.

## Vision / pose estimation notes

- `SwerveDrivePoseEstimator` covariance matrices are tuned in `SwerveDrive.init()` — don't set them elsewhere.
- Vision timestamps must use `Timer.getTimestamp()` consistently — see `Vision.sense()`.
- `FieldConstants.FIELD` is currently `WELDED` (2026 Rebuilt Welded field).
