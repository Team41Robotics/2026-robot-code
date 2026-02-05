package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.test.drive.TurnPIDTestCommand;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	public Robot() {
		RobotContainer.robot = this;
	}

	@Override
	public void robotInit() {
		if (isReal()) {
			Logger.addDataReceiver(new NT4Publisher());
			// Logger.addDataReceiver(new WPILOGWriter("/V/logs"));
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		Logger.start();

		RobotContainer.init();

		enableLiveWindowInTest(false);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		if (RobotContainer.autonomousCommand != null) {
			RobotContainer.autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (RobotContainer.autonomousCommand != null) {
			RobotContainer.autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		new TurnPIDTestCommand().schedule();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
