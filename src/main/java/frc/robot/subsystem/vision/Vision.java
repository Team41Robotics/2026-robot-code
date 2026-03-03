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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
	public boolean preferConstrainedPnP = false;

	public void init() {
		for (int i = 0; i < nCams; i++) {
			VisionHW cam = cameras[i];
			inputs[i] = new VisionInputsAutoLogged();
			poseEsts[i] = new PhotonPoseEstimator(TAG_LAYOUT, cam.camPos);
			decodePackets[i] = new ReusablePacket(0); // Pre-allocate decode packets
			cam.init();
		}
		sense();
	}

	public void sense() {
		Logger.recordOutput("/Vision/enabled", enabled);

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

				if (constrainedPnPpose.isPresent() && preferConstrainedPnP) {
					visionPose = constrainedPnPpose.get().estimatedPose.toPose2d();
					method = "constrainedPnP";
				} else if (coprocPnPpose.isPresent() && result.targets.size() >= 2) {
					visionPose = coprocPnPpose.get().estimatedPose.toPose2d();
					method = "multiTag";
				} else if (pnpDistTrigPose.isPresent()) {
					visionPose = pnpDistTrigPose.get().estimatedPose.toPose2d();
					method = "pnpDistTrig";
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
							cov = Util.buildCov(0.6, 0.6, 0.8);
							break;
						default:
							cov = Util.buildCov(1.0, 1.0, 1.0);
							break;
					}
					drive.poseEst.addVisionMeasurement(visionPose, result.getTimestampSeconds(), cov);
					Logger.recordOutput("/Vision/" + cam.name + "/estimatedPose", visionPose);
				}
			}
		}
	}
}
