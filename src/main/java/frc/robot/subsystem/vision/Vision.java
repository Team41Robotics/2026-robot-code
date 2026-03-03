package frc.robot.subsystem.vision;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Util;
import frc.robot.subsystem.drive.SwerveDrive;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.ReusablePacket;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
	public VisionHW[] cameras = new VisionHW[] {
		new VisionHW(
				"DuckyNE",
				new Transform3d(
						new Translation3d(SwerveDrive.ROBOT_LEN / 2, -SwerveDrive.ROBOT_WID / 2, 0.17),
						new Rotation3d(0, -20. / 180. * PI, 0))),
		new VisionHW(
				"DuckySE",
				new Transform3d(
						new Translation3d(-SwerveDrive.ROBOT_LEN / 2, -SwerveDrive.ROBOT_WID / 2, 0.17),
						new Rotation3d(0, -20. / 180. * PI, PI)))
	};
	public VisionInputsAutoLogged[] inputs = new VisionInputsAutoLogged[cameras.length];
	public PhotonPoseEstimator[] poseEsts = new PhotonPoseEstimator[cameras.length];

	public ReusablePacket[] decodePackets = new ReusablePacket[cameras.length];

	public static final AprilTagFieldLayout TAG_LAYOUT =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public int nCams = cameras.length;
	public boolean enabled = true;
	public boolean enableConstrainedPnP = false;
	public boolean enableMultiTag = true;
	public boolean enablePnpDistTrig = true;

	public void init() {
		for (int i = 0; i < nCams; i++) {
			VisionHW cam = cameras[i];
			inputs[i] = new VisionInputsAutoLogged();
			poseEsts[i] = new PhotonPoseEstimator(TAG_LAYOUT, cam.camPos);
			decodePackets[i] = new ReusablePacket(0); // Pre-allocate decode packets
			cam.init();
		}
		SmartDashboard.putBoolean("Vision/enableConstrainedPnP", enableConstrainedPnP);
		SmartDashboard.putBoolean("Vision/enableMultiTag", enableMultiTag);
		SmartDashboard.putBoolean("Vision/enablePnpDistTrig", enablePnpDistTrig);
		sense();
	}

	public static boolean sane(Optional<EstimatedRobotPose> est) {
		if (est.isEmpty()) return false;
		Pose3d p = est.get().estimatedPose;
		double x = p.getX(), y = p.getY(), z = p.getZ();
		return x >= 0
				&& x <= FieldConstants.fieldLength
				&& y >= 0
				&& y <= FieldConstants.fieldWidth
				&& z >= -2.0
				&& z <= 3.0;
	}

	public void sense() {
		Logger.recordOutput("/Vision/enabled", enabled);
		enableConstrainedPnP = SmartDashboard.getBoolean("Vision/enableConstrainedPnP", enableConstrainedPnP);
		enableMultiTag = SmartDashboard.getBoolean("Vision/enableMultiTag", enableMultiTag);
		enablePnpDistTrig = SmartDashboard.getBoolean("Vision/enablePnpDistTrig", enablePnpDistTrig);

		for (int i = 0; i < nCams; i++) {
			VisionHW cam = cameras[i];
			VisionInputsAutoLogged input = inputs[i];

			cam.sense(input);
			Logger.processInputs("/Vision/" + cam.name, input);
			decodePackets[i].setData(input.data);
			List<PhotonPipelineResult> results = decodePackets[i].decodeList(PhotonPipelineResult.photonStruct);

			poseEsts[i].addHeadingData(Timer.getTimestamp(), drive.rot);

			for (int j = 0; j < results.size(); j++) {
				PhotonPipelineResult result = results.get(j);
				Pose2d visionPose = null;
				String method = "none";

				if (result.targets.isEmpty()) {
					Logger.recordOutput("/Vision/" + cam.name + "/method", "none");
					continue;
				}

				Optional<EstimatedRobotPose> constrainedPnPpose;
				try {
					constrainedPnPpose = poseEsts[i].estimateConstrainedSolvepnpPose(
							result,
							cam.cam.getCameraMatrix().orElseThrow(),
							cam.cam.getDistCoeffs().orElseThrow(),
							new Pose3d(drive.pose),
							false,
							1);
				} catch (Exception e) {
					constrainedPnPpose = Optional.empty();
					System.out.println("Constrained PnP failed for " + cam.name + ": " + e.getMessage());
				}
				Optional<EstimatedRobotPose> coprocPnPpose;
				try {
					coprocPnPpose = poseEsts[i].estimateCoprocMultiTagPose(result);
				} catch (Exception e) {
					coprocPnPpose = Optional.empty();
					System.out.println("Co-processor PnP failed for " + cam.name + ": " + e.getMessage());
				}
				Optional<EstimatedRobotPose> pnpDistTrigPose;
				try {
					pnpDistTrigPose = poseEsts[i].estimatePnpDistanceTrigSolvePose(result);
				} catch (Exception e) {
					pnpDistTrigPose = Optional.empty();
					System.out.println("PnP Distance+Trig failed for " + cam.name + ": " + e.getMessage());
				}

				Logger.recordOutput(
						"/Vision/" + cam.name + "/constrainedPnPPose",
						constrainedPnPpose.isPresent() ? constrainedPnPpose.get().estimatedPose : null);
				Logger.recordOutput(
						"/Vision/" + cam.name + "/coprocPnPPose",
						coprocPnPpose.isPresent() ? coprocPnPpose.get().estimatedPose : null);
				Logger.recordOutput(
						"/Vision/" + cam.name + "/pnpDistTrigPose",
						pnpDistTrigPose.isPresent() ? pnpDistTrigPose.get().estimatedPose : null);

				if (sane(pnpDistTrigPose) && enablePnpDistTrig) {
					visionPose = pnpDistTrigPose.get().estimatedPose.toPose2d();
					method = "pnpDistTrig";
				} else if (sane(constrainedPnPpose) && enableConstrainedPnP) {
					visionPose = constrainedPnPpose.get().estimatedPose.toPose2d();
					method = "constrainedPnP";
				} else if (sane(coprocPnPpose) && result.targets.size() >= 2 && enableMultiTag) {
					visionPose = coprocPnPpose.get().estimatedPose.toPose2d();
					method = "multiTag";
				}

				Logger.recordOutput("/Vision/" + cam.name + "/nTargets", result.targets.size());
				Logger.recordOutput("/Vision/" + cam.name + "/method", method);

				if (visionPose != null && enabled) {
					// TUNEME. vision measurement covariance (tune)
					Matrix<N3, N1> cov;
					switch (method) {
						case "constrainedPnP":
							cov = Util.buildCov(0.5, 0.5, 0.7);
							break;
						case "multiTag":
							cov = Util.buildCov(0.9, 0.9, 1.2);
							break;
						case "pnpDistTrig":
							cov = Util.buildCov(0.6, 0.6, 6.0);
							break;
						default:
							throw new IllegalStateException("Unexpected vision method: " + method);
					}
					drive.poseEst.addVisionMeasurement(visionPose, result.getTimestampSeconds(), cov);
					Logger.recordOutput("/Vision/" + cam.name + "/estimatedPose", visionPose);
				}
			}
		}
	}
}
