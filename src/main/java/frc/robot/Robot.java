package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	public MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
	public List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
	public long prevGcCount = 0;
	public long prevGcTime = 0;

	public Robot() {
		super();
		RobotContainer.robot = this;
	}

	@Override
	public void robotInit() {
		if (isReal()) {
			Logger.addDataReceiver(new NT4Publisher());
			Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		LoggedPowerDistribution.getInstance(7, ModuleType.kRev); // FIXME.

		Logger.start();

		RobotContainer.init();

		enableLiveWindowInTest(false);
	}

	@Override
	public void robotPeriodic() {
		RobotContainer.periodic();

		logGCStatistics();
	}

	public void logGCStatistics() {
		MemoryUsage heapUsage = memoryBean.getHeapMemoryUsage();
		Logger.recordOutput("/GC/HeapUsedMB", heapUsage.getUsed() / 1_048_576.0);
		Logger.recordOutput("/GC/HeapCommittedMB", heapUsage.getCommitted() / 1_048_576.0);
		Logger.recordOutput("/GC/HeapMaxMB", heapUsage.getMax() / 1_048_576.0);
		Logger.recordOutput("/GC/HeapUsedPercent", 100.0 * heapUsage.getUsed() / heapUsage.getMax());

		MemoryUsage nonHeapUsage = memoryBean.getNonHeapMemoryUsage();
		Logger.recordOutput("/GC/NonHeapUsedMB", nonHeapUsage.getUsed() / 1_048_576.0);

		long totalGcCount = 0;
		long totalGcTime = 0;
		for (GarbageCollectorMXBean gcBean : gcBeans) {
			long count = gcBean.getCollectionCount();
			long time = gcBean.getCollectionTime();
			if (count >= 0) totalGcCount += count;
			if (time >= 0) totalGcTime += time;

			String name = gcBean.getName().replace(" ", "");
			Logger.recordOutput("/GC/Collector/" + name + "/Count", count);
			Logger.recordOutput("/GC/Collector/" + name + "/TimeMS", time);
		}

		Logger.recordOutput("/GC/TotalCollections", totalGcCount);
		Logger.recordOutput("/GC/TotalCollectionTimeMS", totalGcTime);
		Logger.recordOutput("/GC/CollectionsSinceLast", totalGcCount - prevGcCount);
		Logger.recordOutput("/GC/CollectionTimeMSSinceLast", totalGcTime - prevGcTime);

		prevGcCount = totalGcCount;
		prevGcTime = totalGcTime;
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
			CommandScheduler.getInstance().schedule(RobotContainer.autonomousCommand);
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
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
