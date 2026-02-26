package frc.robot.subsystem.vision;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.ReusablePacket;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
	public VisionHW[] cameras = new VisionHW[] {new VisionHW("TODO", new Transform3d())};
	public VisionInputsAutoLogged[] inputs = new VisionInputsAutoLogged[cameras.length];
	public PhotonPoseEstimator[] poseEsts = new PhotonPoseEstimator[cameras.length];

	public ReusablePacket[] decodePackets = new ReusablePacket[cameras.length];

	public static final AprilTagFieldLayout TAG_LAYOUT =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public int nCams = cameras.length;
	public boolean enabled = true;

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

			poseEsts[i].addHeadingData(Timer.getTimestamp(), drive.pose.getRotation());

			for (int j = 0; j < results.size(); j++) {
				PhotonPipelineResult result = results.get(j);
				Pose2d visionPose = null;
				String method = "none";

				Optional<EstimatedRobotPose> estPose = poseEsts[i].estimateConstrainedSolvepnpPose(
						result,
						cam.cam.getCameraMatrix().orElseThrow(),
						cam.cam.getDistCoeffs().orElseThrow(),
						new Pose3d(drive.pose),
						false,
						1);
				if (estPose.isPresent()) {
					visionPose = estPose.get().estimatedPose.toPose2d();
					method = "constrainedPnP";
				} else {
					estPose = poseEsts[i].estimateCoprocMultiTagPose(result);
					if (estPose.isPresent() && result.targets.size() >= 2) {
						visionPose = estPose.get().estimatedPose.toPose2d();
						method = "multiTag";
					} else {
						estPose = poseEsts[i].estimatePnpDistanceTrigSolvePose(result);
						if (estPose.isPresent()) {
							visionPose = estPose.get().estimatedPose.toPose2d();
							method = "pnpDistTrig";
						}
					}
				}

				Logger.recordOutput("/Vision/" + cam.name + "/nTargets", result.targets.size());
				Logger.recordOutput("/Vision/" + cam.name + "/method", method);

				if (visionPose != null && enabled) {
					drive.poseEst.addVisionMeasurement(
							visionPose, result.getTimestampSeconds(), VecBuilder.fill(0.75, 0.75, 0.9));
					Logger.recordOutput("/Vision/" + cam.name + "/estimatedPose", visionPose);
				}
			}
		}
	}
}
