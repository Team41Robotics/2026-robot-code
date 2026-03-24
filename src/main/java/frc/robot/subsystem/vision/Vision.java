package frc.robot.subsystem.vision;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
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
				"KimmySE",
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

	public int nCams = cameras.length;
	public boolean enabled = true;
	public LoggedNetworkBoolean enableMultiTag = new LoggedNetworkBoolean("/Vision/enableMultiTag", true);
	public LoggedNetworkBoolean enablePnpDistTrig = new LoggedNetworkBoolean("/Vision/enablePnpDistTrig", false);

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

		new Trigger(() -> robot.isEnabled()).onTrue(new InstantCommand(() -> {
			enableMultiTag.set(false);
			enablePnpDistTrig.set(true);
		}));

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

				Pose3d[] seenTagPoses3d = result.targets.stream()
						.map(t -> TAG_LAYOUT.getTagPose(t.fiducialId))
						.filter(Optional::isPresent)
						.map(Optional::get)
						.toArray(Pose3d[]::new);
				Logger.recordOutput("/Vision/" + cam.name + "/seenTags3d", seenTagPoses3d);

				// --- Multi-tag pnpDistTrig fusion ---
				// Run pnpDistTrig on each tag individually, fuse with 1/d² weighting
				Pose2d fusedTrigPose = null;
				double fusedTrigWeightSum = 0;
				double fusedX = 0, fusedY = 0, fusedSinT = 0, fusedCosT = 0;
				int trigSuccessCount = 0;
				for (PhotonTrackedTarget target : result.targets) {
					PhotonPipelineResult singleResult =
							new PhotonPipelineResult(result.metadata, List.of(target), Optional.empty());
					Optional<EstimatedRobotPose> singlePose =
							poseEsts[i].estimatePnpDistanceTrigSolvePose(singleResult);
					if (sane(singlePose)) {
						double d =
								target.getBestCameraToTarget().getTranslation().getNorm();
						double w = 1.0 / (d * d);
						Pose2d p = singlePose.get().estimatedPose.toPose2d();
						fusedX += p.getX() * w;
						fusedY += p.getY() * w;
						fusedSinT += sin(p.getRotation().getRadians()) * w;
						fusedCosT += cos(p.getRotation().getRadians()) * w;
						fusedTrigWeightSum += w;
						trigSuccessCount++;
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
				Optional<EstimatedRobotPose> coprocPnPpose;
				try {
					coprocPnPpose = poseEsts[i].estimateCoprocMultiTagPose(result);
				} catch (Exception e) {
					coprocPnPpose = Optional.empty();
				}

				String prefix = "/Vision/" + cam.name;

				Logger.recordOutput(prefix + "/nTargets", result.targets.size());

				// Log per-tag info: IDs and distances
				int[] tagIds =
						result.targets.stream().mapToInt(t -> t.fiducialId).toArray();
				double[] tagDists = result.targets.stream()
						.mapToDouble(
								t -> t.getBestCameraToTarget().getTranslation().getNorm())
						.toArray();
				Logger.recordOutput(prefix + "/tagIds", tagIds);
				Logger.recordOutput(prefix + "/tagDists", tagDists);

				// Fuse sane methods into pose estimator independently
				ArrayList<String> methods = new ArrayList<>();

				// pnpDistTrig: proper error prop — σ² = base / Σ(1/dᵢ²)
				if (sane(pnpDistTrigPose) && enablePnpDistTrig.get() && enabled) {
					Pose2d pose = pnpDistTrigPose.get().estimatedPose.toPose2d();
					double distScale = 1.0 / fusedTrigWeightSum;
					double xyStd = 0.04 * distScale;
					double thetaStd = 4.0 * distScale;
					drive.poseEst.addVisionMeasurement(
							pose, result.getTimestampSeconds(), Util.buildCov(xyStd, xyStd, thetaStd));
					methods.add("pnpDistTrig");
					Logger.recordOutput(prefix + "/pnpDistTrig/pose", pose);
					Logger.recordOutput(prefix + "/pnpDistTrig/xyStd", xyStd);
					Logger.recordOutput(prefix + "/pnpDistTrig/thetaStd", thetaStd);
					Logger.recordOutput(prefix + "/pnpDistTrig/trigCount", trigSuccessCount);
					Logger.recordOutput(prefix + "/pnpDistTrig/weightSum", fusedTrigWeightSum);
				}

				// multiTag: heading correction only — loose XY, tight theta
				if (sane(coprocPnPpose) && result.targets.size() >= 2 && enableMultiTag.get() && enabled) {
					Pose2d pose = coprocPnPpose.get().estimatedPose.toPose2d();
					double distScale = result.targets.stream()
							.mapToDouble(t -> {
								double d = t.getBestCameraToTarget()
										.getTranslation()
										.getNorm();
								return d * d;
							})
							.average()
							.orElse(16.0);
					double tagScale = 1.0 / sqrt(result.targets.size());
					double xyStd = 1.0 * distScale * tagScale;
					double thetaStd = 0.01 * distScale * tagScale;
					drive.poseEst.addVisionMeasurement(
							pose, result.getTimestampSeconds(), Util.buildCov(xyStd, xyStd, thetaStd));
					methods.add("multiTag");
					Logger.recordOutput(prefix + "/multiTag/pose", pose);
					Logger.recordOutput(prefix + "/multiTag/xyStd", xyStd);
					Logger.recordOutput(prefix + "/multiTag/thetaStd", thetaStd);
				}

				Logger.recordOutput(prefix + "/methods", methods.isEmpty() ? "none" : String.join("+", methods));
			}
		}
	}
}
