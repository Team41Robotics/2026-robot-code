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
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystem.drive.SwerveDrive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.ReusablePacket;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
	public VisionHW[] cameras = new VisionHW[] {
		new VisionHW(
				"DuckyNE",
				new Transform3d(
						new Translation3d(SwerveDrive.ROBOT_LEN / 2, -SwerveDrive.ROBOT_WID / 2, 0.17),
						new Rotation3d(0, -20. / 180. * PI, 0)),
				"Ducky.json"),
		new VisionHW(
				"DuckySE",
				new Transform3d(
						new Translation3d(-SwerveDrive.ROBOT_LEN / 2, -SwerveDrive.ROBOT_WID / 2, 0.17),
						new Rotation3d(0, -20. / 180. * PI, PI)),
				"Kimmy.json")
	};
	public VisionInputsAutoLogged[] inputs = new VisionInputsAutoLogged[cameras.length];
	public PhotonPoseEstimator[] poseEsts = new PhotonPoseEstimator[cameras.length];

	public ReusablePacket[] decodePackets = new ReusablePacket[cameras.length];

	public static final AprilTagFieldLayout TAG_LAYOUT =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public static final double CPNP_MAX_TAG_DIST = 6.0; // meters — filter tags beyond this for constrainedPnP

	public int nCams = cameras.length;
	public boolean enabled = true;
	public LoggedNetworkBoolean enableConstrainedPnP = new LoggedNetworkBoolean("/Vision/enableConstrainedPnP", true);
	public LoggedNetworkBoolean enableMultiTag = new LoggedNetworkBoolean("/Vision/enableMultiTag", true);
	public LoggedNetworkBoolean enablePnpDistTrig = new LoggedNetworkBoolean("/Vision/enablePnpDistTrig", true);
	public LoggedNetworkBoolean enableHeading = new LoggedNetworkBoolean("/Vision/enableHeading", false);

	public void init() {
		try {
			System.loadLibrary("photontargetingJNI");
		} catch (Throwable e) {
			System.out.println("Warning: could not load photontargetingJNI: " + e.getMessage());
		}
		for (int i = 0; i < nCams; i++) {
			VisionHW cam = cameras[i];
			inputs[i] = new VisionInputsAutoLogged();
			poseEsts[i] = new PhotonPoseEstimator(TAG_LAYOUT, cam.camPos);
			decodePackets[i] = new ReusablePacket(0); // Pre-allocate decode packets
			cam.init();
		}
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

		if (!Robot.isReal() && robot.isTeleopEnabled()) enableHeading.set(true);

		for (int i = 0; i < nCams; i++) {
			VisionHW cam = cameras[i];
			VisionInputsAutoLogged input = inputs[i];

			cam.sense(input);
			Logger.processInputs("/Vision/" + cam.name, input);
			decodePackets[i].setData(input.data);
			List<PhotonPipelineResult> results = decodePackets[i].decodeList(PhotonPipelineResult.photonStruct);
			// System.out.println("Camera " + cam.name + " sees " + results.size() + " targets");

			poseEsts[i].addHeadingData(Timer.getTimestamp(), drive.rot);

			for (int j = 0; j < results.size(); j++) {
				PhotonPipelineResult result = results.get(j);

				if (result.targets.isEmpty()) {
					Logger.recordOutput("/Vision/" + cam.name + "/method", "none");
					continue;
				}

				// Log seen AprilTag poses for AdvantageScope
				Pose3d[] seenTagPoses3d = result.targets.stream()
						.map(t -> TAG_LAYOUT.getTagPose(t.fiducialId))
						.filter(Optional::isPresent)
						.map(Optional::get)
						.toArray(Pose3d[]::new);
				Pose2d[] seenTagPoses2d = java.util.Arrays.stream(seenTagPoses3d)
						.map(Pose3d::toPose2d)
						.toArray(Pose2d[]::new);
				Logger.recordOutput("/Vision/" + cam.name + "/seenTags3d", seenTagPoses3d);
				Logger.recordOutput("/Vision/" + cam.name + "/seenTags", seenTagPoses2d);

				// --- Multi-tag pnpDistTrig fusion ---
				// Run pnpDistTrig on each tag individually, fuse with 1/d² weighting
				Pose2d fusedTrigPose = null;
				double fusedTrigWeightSum = 0;
				double fusedX = 0, fusedY = 0, fusedSinT = 0, fusedCosT = 0;
				int trigSuccessCount = 0;
				for (PhotonTrackedTarget target : result.targets) {
					try {
						PhotonPipelineResult singleResult =
								new PhotonPipelineResult(result.metadata, List.of(target), Optional.empty());
						Optional<EstimatedRobotPose> singlePose =
								poseEsts[i].estimatePnpDistanceTrigSolvePose(singleResult);
						if (sane(singlePose)) {
							double d = target.getBestCameraToTarget()
									.getTranslation()
									.getNorm();
							double w = 1.0 / (d * d);
							Pose2d p = singlePose.get().estimatedPose.toPose2d();
							fusedX += p.getX() * w;
							fusedY += p.getY() * w;
							fusedSinT += sin(p.getRotation().getRadians()) * w;
							fusedCosT += cos(p.getRotation().getRadians()) * w;
							fusedTrigWeightSum += w;
							trigSuccessCount++;
						}
					} catch (Exception e) {
						// skip this tag
					}
				}
				Optional<EstimatedRobotPose> pnpDistTrigPose;
				if (trigSuccessCount > 0) {
					fusedTrigPose = new Pose2d(
							fusedX / fusedTrigWeightSum,
							fusedY / fusedTrigWeightSum,
							new edu.wpi.first.math.geometry.Rotation2d(
									fusedCosT / fusedTrigWeightSum, fusedSinT / fusedTrigWeightSum));
					// Wrap as EstimatedRobotPose for logging consistency
					pnpDistTrigPose = Optional.of(new EstimatedRobotPose(
							new Pose3d(fusedTrigPose),
							result.getTimestampSeconds(),
							result.targets,
							org.photonvision.PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE));
				} else {
					pnpDistTrigPose = Optional.empty();
				}
				Logger.recordOutput("/Vision/" + cam.name + "/trigFusedCount", trigSuccessCount);

				// --- Filtered + distance-weighted constrainedPnP ---
				Optional<EstimatedRobotPose> constrainedPnPpose;
				if (input.cameraMatrix.length == 0 || input.distCoeffs.length == 0) {
					constrainedPnPpose = Optional.empty();
				} else {
					// Filter out far tags, duplicate close tags for distance weighting
					ArrayList<PhotonTrackedTarget> weightedTargets = new ArrayList<>();
					int uniqueCloseTagCount = 0;
					for (PhotonTrackedTarget target : result.targets) {
						double d = target.getBestCameraToTarget().getTranslation().getNorm();
						if (d > CPNP_MAX_TAG_DIST) continue;
						uniqueCloseTagCount++;
						int copies = (int) min(CPNP_MAX_TAG_DIST, max(1, floor(CPNP_MAX_TAG_DIST / d)));
						for (int k = 0; k < copies; k++) weightedTargets.add(target);
					}
					Logger.recordOutput("/Vision/" + cam.name + "/cpnpUniqueTags", uniqueCloseTagCount);
					Logger.recordOutput("/Vision/" + cam.name + "/cpnpWeightedTags", weightedTargets.size());

					if (uniqueCloseTagCount >= 2) {
						try {
							PhotonPipelineResult filteredResult =
									new PhotonPipelineResult(result.metadata, weightedTargets, Optional.empty());
							// Use fused trig pose as seed when available, otherwise fall back to pose estimator
							Pose3d seedPose;
							if (fusedTrigPose != null) {
								seedPose = new Pose3d(fusedTrigPose);
							} else {
								seedPose = new Pose3d(drive.poseEst
										.sampleAt(result.getTimestampSeconds())
										.orElse(drive.pose));
							}
							constrainedPnPpose = poseEsts[i].estimateConstrainedSolvepnpPose(
									filteredResult,
									new Matrix<>(N3.instance, N3.instance, input.cameraMatrix),
									new Matrix<>(N8.instance, N1.instance, input.distCoeffs),
									seedPose,
									!enableHeading.get(),
									1e3);
						} catch (Exception e) {
							constrainedPnPpose = Optional.empty();
						}
					} else {
						constrainedPnPpose = Optional.empty();
					}
				}

				Optional<EstimatedRobotPose> coprocPnPpose;
				try {
					coprocPnPpose = poseEsts[i].estimateCoprocMultiTagPose(result);
				} catch (Exception e) {
					coprocPnPpose = Optional.empty();
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

				Logger.recordOutput("/Vision/" + cam.name + "/nTargets", result.targets.size());

				// avg(d²) not (avg d)² — far tags dominate error
				double distScale = result.targets.stream()
						.mapToDouble(t -> {
							double d = t.getBestCameraToTarget()
									.getTranslation()
									.getNorm();
							return d * d;
						})
						.average()
						.orElse(16.0);
				Logger.recordOutput("/Vision/" + cam.name + "/avgDistSq", distScale);

				// Fuse all sane methods into pose estimator independently
				ArrayList<String> methods = new ArrayList<>();

				if (sane(constrainedPnPpose) && enableConstrainedPnP.get() && enabled) {
					Pose2d pose = constrainedPnPpose.get().estimatedPose.toPose2d();
					double tagScale = 1.0 / sqrt(result.targets.size());
					double xyStd = 0.08 * distScale * tagScale;
					double thetaStd = 0.5 * distScale * tagScale;
					drive.poseEst.addVisionMeasurement(
							pose, result.getTimestampSeconds(), Util.buildCov(xyStd, xyStd, thetaStd));
					Logger.recordOutput("/Vision/" + cam.name + "/constrainedPnPCovXY", xyStd);
					methods.add("constrainedPnP");
				}

				if (sane(pnpDistTrigPose) && enablePnpDistTrig.get() && enabled) {
					Pose2d pose = pnpDistTrigPose.get().estimatedPose.toPose2d();
					double tagScale = 1.0 / sqrt(trigSuccessCount);
					double xyStd = 0.04 * distScale * tagScale;
					double thetaStd = 4.0 * distScale * tagScale;
					drive.poseEst.addVisionMeasurement(
							pose, result.getTimestampSeconds(), Util.buildCov(xyStd, xyStd, thetaStd));
					Logger.recordOutput("/Vision/" + cam.name + "/pnpDistTrigCovXY", xyStd);
					methods.add("pnpDistTrig");
				}

				if (sane(coprocPnPpose) && result.targets.size() >= 3 && enableMultiTag.get() && enabled) {
					Pose2d pose = coprocPnPpose.get().estimatedPose.toPose2d();
					double tagScale = 1.0 / sqrt(result.targets.size());
					double xyStd = 0.12 * distScale * tagScale;
					double thetaStd = (enableHeading.get() ? 0.8 : 0.01) * distScale * tagScale;
					drive.poseEst.addVisionMeasurement(
							pose, result.getTimestampSeconds(), Util.buildCov(xyStd, xyStd, thetaStd));
					Logger.recordOutput("/Vision/" + cam.name + "/multiTagCovXY", xyStd);
					methods.add("multiTag");
				}

				String methodStr = methods.isEmpty() ? "none" : String.join("+", methods);
				Logger.recordOutput("/Vision/" + cam.name + "/methods", methodStr);
				Logger.recordOutput("/Vision/" + cam.name + "/methodCount", methods.size());
			}
		}
	}
}
