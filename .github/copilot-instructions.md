## What this repo is (short)

This is a WPILib-based robot codebase (Java) built around a sense -> schedule -> actuate loop. It uses Littleton Robotics' logging/junction tooling, CTRE Phoenix 6 devices, PhotonVision for vision pose estimation, and Swerve drive kinematics/estimation.

## High-level architecture (what matters to an AI)

- Robot lifecycle: `Robot` extends `LoggedRobot` and delegates initialization and periodic work to `RobotContainer`.
- Main loop pattern (in `RobotContainer.periodic()`):
  1. call `imu.sense()` / `drive.sense()` / `vision.sense()` (read sensors)
  2. run `CommandScheduler.getInstance().run()` (commands are scheduled/executed)
  3. call `drive.actuate()` (write actuators)

  When adding or changing subsystems, follow this split: low-level HW classes provide `init()`, `sense()` and `actuate()` hooks; higher-level subsystem classes orchestrate those.

## Key files to inspect first

- `src/main/java/frc/robot/Robot.java` — robot lifecycle + Logger setup (simulation vs real behavior). Note replay support using WPILOGReader.
- `src/main/java/frc/robot/RobotContainer.java` — central wiring: static subsystems, `init()` and `periodic()` order, default commands, SmartDashboard calibration entries.
- `src/main/java/frc/robot/FieldConstants.java` — field layout, AprilTag JSON lookup (deploy/apriltags). Important when changing vision/tag logic.
- `src/main/java/frc/robot/subsystem/drive/SwerveDrive.java` and `SwerveHW.java` — example of module configs, kinematics, estimator, logging and the hardware pattern.
- `src/main/java/frc/robot/subsystem/vision/Vision.java` — PhotonVision + `PhotonPoseEstimator` usage and how vision measurements are fed into the drive pose estimator.

## Coding patterns & conventions (concrete)

- Sense/Actuate separation: Subsystems implement `init()`, `sense()` (populate inputs / Logger.processInputs), and `actuate()` (write outputs / Logger.recordOutput). See `SwerveDrive`, `SwerveHW`, and `Vision` for examples.
- Static wiring: `RobotContainer` exposes static subsystem instances (e.g., `RobotContainer.drive`, `RobotContainer.imu`) and is statically imported in many subsystem files (`import static frc.robot.RobotContainer.*;`). When editing code, prefer referencing these statics rather than creating new instances.
- No encapsulation: Subsystem fields are often public for direct access (e.g., `SwerveDrive.pose`, `Vision.cameras`) to simplify data flow and logging.
- Logging conventions: use `Logger.processInputs("/Subsystem/Name", inputs)` for inputs and `Logger.recordOutput("/Path/entry", value)` for outputs (examples in `SwerveDrive.sense()` and `SwerveHW.actuate()`). Keep the path consistent with subsystem names. Log as much as possible for replay/debugging, especially sensor readings and control setpoints.
- Hardware IDs/configs: Swerve module IDs and angle offsets live in `SwerveDrive.configs` (see the `SwerveModuleConfiguration[]` array). To change hardware mapping, update those entries — don't scatter IDs through code.
- CAN bus: `RobotContainer.driveBus` is configured once; CTRE Phoenix 6 components are created with that bus (see `SwerveHW.init`). Use the `driveBus` instance when instantiating hardware.
- Field/AprilTag JSONs: deployed JSONs are in `src/main/deploy/apriltags/...`. `FieldConstants.AprilTagLayoutType` loads them via WPILib's deploy directory at runtime; change files there for simulation/replay.

## Naming conventions (new repository standard)

Follow these rules when adding or renaming variables, constants, and fields. The goal is concise, math-friendly names plus a clear, machine-parsable prefix scheme for measured vs. commanded values.

- Short math-style locals are preferred in algorithms and commands:
  - velocities: `v_x`, `v_y` (field-relative components), `v` or `mag` for magnitude
  - angles/heading: `theta`
  - angular velocity: `w` (omega)
  - use `vx`, `vy`, `w` in one-letter math contexts for clarity and compact expressions

- Swerve module / subsystem fields follow this measured/target/setpoint convention:
  - Measured (sensor) values: simple, short names (no prefix). Examples: `angle`, `vel`, `drivePos`, `state` (for a SwerveModuleState)
  - Target/command inputs from higher-level code: `targetVel`, `targetAng` (these are what you asked the module to achieve)
  - Controller setpoints / profiler outputs: `setpointVel`, `setpointAng` (these are internal controller/trapezoid outputs)
  - Use `state` for the WPILib kinematic object (e.g., `SwerveModuleState state`) and reserve `target`/`setpoint` names for numeric values produced/consumed by control loops.

- Constants naming:
  - Use an ALL_CAPS subsystem prefix, underscore, then the conventional WPILib-style lowercase `k` suffix for gains. Examples:
    - Feedforward: `DRIVE_kS`, `DRIVE_kV`, `DRIVE_kA`, `TURN_kS`, `TURN_kV`, `TURN_kA`
    - PID gains: `DRIVE_kP`, `TURN_kP`, `TURN_kD` (note the lowercase `k` then uppercase term)
  - Use `MAX_VEL` and `MAX_W` for physical limits (meters/sec and rad/sec respectively).

- Trapezoid/profile conventions:
  - Make `TrapezoidProfile.Constraints` objects constants when they represent fixed tuning values for a controller: `public static final Constraints TURN_CONSTRAINTS = new Constraints(maxVel, maxAccel);`
  - Do NOT make `PIDController` objects static/constants. Keep `PIDController` and `TrapezoidProfile` instances as non-static instance fields (they have internal state and should be per-command or per-subsystem).

- Logger / API change guidance when renaming:
  - When renaming fields used by logging or by other classes (for example `drive.poseEst` → `drive.poseEstimator` or vice versa), update all static imports and Logger paths. Logger keys should remain consistent when possible.

Examples (recommended style):

- SwerveModule fields:
  - public SwerveModuleState state; // measured
  - public double angle; // measured
  - public double vel; // measured
  - public double drivePos; // measured encoder
  - public double targetVel; // commanded
  - public double targetAng; // commanded
  - public State setpointAng; // trapezoid output

- Constants in `SwerveModule`/`SwerveHW`:
  - public static final double DRIVE_kV = 1.23;
  - public static final double DRIVE_kS = 0.12;
  - public static final double DRIVE_kA = 0.01;
  - public static final double MAX_VEL = 6.0;
  - public static final double MAX_W = MAX_VEL / hypot(ROBOT_LEN/2, ROBOT_WID/2);

These conventions strike a balance between compact math-like code (easier to read in control/math contexts) while preserving explicit, machine-friendly field names for logging and cross-class references.

## Build / test / formatting (quick commands)

On Windows (project includes `gradlew.bat`):

 - Format: `gradlew.bat spotlessApply` (project has Spotless config — see workspace tasks)
 - Build: `gradlew.bat build`

Note: The repo uses Littleton's `junction` logging. Running in simulation sets the Logger to replay mode (see `Robot.robotInit()`).

## Adding a new subsystem (checklist)

1. Create a HW class with `init()`, `sense(Inputs)`, and `actuate(Inputs, ...)` where `Inputs` is an AutoLogged POJO (pattern in `vision.*` and `drive.*`).
2. Add subsystem instance to `RobotContainer` and call `init()` there.
3. Add `sense()` calls in `RobotContainer.periodic()` and `actuate()` calls after `CommandScheduler.run()` where appropriate.
4. Wire Logger calls using the `/Subsystem/...` paths used elsewhere.
5. Add SmartDashboard entries (if interactive tuning is needed) from `RobotContainer.init()`.

## Vision & Estimation gotchas

- `Vision` uses `PhotonPoseEstimator` with multiple fallback estimation methods (see `Vision.sense()`). When changing pose fusion, keep measurement timestamps consistent (they use `Timer.getTimestamp()` in specific places).
- The drive's `SwerveDrivePoseEstimator` is seeded in `SwerveDrive.init()` with covariance matrices — tune these there, not ad-hoc elsewhere.

## Where to look for external / vendor info

- `vendordeps/` contains JSON metadata for libraries used. The project uses CTRE Phoenix 6, PhotonVision, Littleton/junction, WPILib, and Lombok.

## PR / patch guidance for AI

- Keep changes small and focused
- Preserve logger keys and subsystem public APIs (many components rely on static `RobotContainer` references). When renaming a subsystem, update every static import and the logging paths.
- If changing hardware IDs or CAN config, update `SwerveDrive.configs` and `RobotContainer.driveBus` in a single commit and annotate why.

---

If anything looks incomplete or you want me to add short examples (e.g., the exact Inputs POJO pattern or a template for a new subsystem), tell me which area to expand and I'll iterate.
