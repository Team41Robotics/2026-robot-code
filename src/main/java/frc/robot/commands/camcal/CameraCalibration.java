package frc.robot.commands.camcal;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

@SuppressWarnings("static-access")
public class CameraCalibration extends Command {

	public static final double FORWARD_SPEED = 0.5;
	public static final double FORWARD_DISTANCE = 1.2;
	public static final double PAUSE_DURATION = 0.5;
	public static final double SPIN_SPEED = 0.3;
	public static final double SPIN_ANGLE = toRadians(370);
	public static final int CAMERA_INDEX = 0;

	public enum Phase {
		FORWARD,
		PAUSE,
		SPIN,
		DONE
	}

	public static class Sample {
		public String phase;
		public double timestamp;
		public double imuYaw;
		public double odomX, odomY;
		public double tx, ty, tz;
		public double qw, qx, qy, qz;
	}

	public Phase phase;
	public ArrayList<Sample> samples = new ArrayList<>();
	public double startX, startY;
	public double pauseStartTime;
	public double accumulatedAngle;
	public double prevYaw;

	public CameraCalibration() {
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		vision.enabled = false;
		samples.clear();
		phase = Phase.FORWARD;
		startX = drive.pose.getX();
		startY = drive.pose.getY();
		System.out.println("[CamCal] Starting calibration. Drive forward phase.");
	}

	@Override
	public void execute() {
		Logger.recordOutput("/CamCal/phase", phase.name());
		Logger.recordOutput("/CamCal/sampleCount", samples.size());

		collectSamples();

		switch (phase) {
			case FORWARD:
				drive.drive(new ChassisSpeeds(FORWARD_SPEED, 0, 0));
				double dx = drive.pose.getX() - startX;
				double dy = drive.pose.getY() - startY;
				if (hypot(dx, dy) >= FORWARD_DISTANCE) {
					drive.drive(new ChassisSpeeds());
					phase = Phase.PAUSE;
					pauseStartTime = Timer.getTimestamp();
					System.out.println("[CamCal] Forward done (" + samples.size() + " samples). Pausing.");
				}
				break;

			case PAUSE:
				drive.drive(new ChassisSpeeds());
				if (Timer.getTimestamp() - pauseStartTime >= PAUSE_DURATION) {
					phase = Phase.SPIN;
					accumulatedAngle = 0;
					prevYaw = imu.yaw;
					System.out.println("[CamCal] Pause done. Spinning.");
				}
				break;

			case SPIN:
				drive.drive(new ChassisSpeeds(0, 0, SPIN_SPEED));
				double deltaYaw = imu.yaw - prevYaw;
				deltaYaw = MathUtil.angleModulus(deltaYaw);
				accumulatedAngle += abs(deltaYaw);
				prevYaw = imu.yaw;
				if (accumulatedAngle >= SPIN_ANGLE) {
					phase = Phase.DONE;
					System.out.println("[CamCal] Spin done (" + samples.size() + " total samples).");
				}
				break;

			case DONE:
				drive.drive(new ChassisSpeeds());
				break;
		}
	}

	public void collectSamples() {
		List<PhotonPipelineResult> results = vision.latestResults[CAMERA_INDEX];
		if (results == null) return;

		String phaseLabel = phase.name();
		for (PhotonPipelineResult result : results) {
			Optional<MultiTargetPNPResult> multiTag = result.getMultiTagResult();
			if (multiTag.isEmpty() || result.targets.size() < 2) continue;

			Transform3d T_ct = multiTag.get().estimatedPose.best;
			Quaternion q = T_ct.getRotation().getQuaternion();

			Sample s = new Sample();
			s.phase = phaseLabel;
			s.timestamp = result.getTimestampSeconds();
			s.imuYaw = imu.yaw;
			s.odomX = drive.pose.getX();
			s.odomY = drive.pose.getY();
			s.tx = T_ct.getX();
			s.ty = T_ct.getY();
			s.tz = T_ct.getZ();
			s.qw = q.getW();
			s.qx = q.getX();
			s.qy = q.getY();
			s.qz = q.getZ();
			samples.add(s);
		}
	}

	@Override
	public boolean isFinished() {
		return phase == Phase.DONE;
	}

	@Override
	public void end(boolean interrupted) {
		drive.drive(new ChassisSpeeds());
		vision.enabled = true;

		if (interrupted) {
			System.out.println("[CamCal] Calibration interrupted!");
			return;
		}

		writeCSV();
	}

	public void writeCSV() {
		String path = Filesystem.getOperatingDirectory().getPath() + "/camcal_data.csv";
		try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
			pw.println("phase,timestamp,imuYaw,odomX,odomY,tx,ty,tz,qw,qx,qy,qz");
			for (Sample s : samples) {
				pw.printf(
						"%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
						s.phase, s.timestamp, s.imuYaw, s.odomX, s.odomY, s.tx, s.ty, s.tz, s.qw, s.qx, s.qy, s.qz);
			}
			System.out.println("[CamCal] Wrote " + samples.size() + " samples to: " + path);
		} catch (IOException e) {
			System.err.println("[CamCal] Failed to write CSV: " + e.getMessage());
		}
	}
}
