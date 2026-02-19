package frc.robot.subsystem.vision;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.ReusablePacket;

public class Vision extends SubsystemBase {
	public VisionHW[] cameras = new VisionHW[] {new VisionHW("TODO", new Transform3d())};
	public VisionInputsAutoLogged[] inputs = new VisionInputsAutoLogged[cameras.length];
	public PhotonPoseEstimator[] poseEsts = new PhotonPoseEstimator[cameras.length];

	public ReusablePacket[] decodePackets = new ReusablePacket[cameras.length];

	public static final AprilTagFieldLayout TAG_LAYOUT =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public final int nCams = cameras.length;

	public void init() {
		// for (int i = 0; i < nCams; i++) {
		// 	VisionHW cam = cameras[i];
		// 	inputs[i] = new VisionInputsAutoLogged();
		// 	poseEsts[i] = new PhotonPoseEstimator(TAG_LAYOUT, cam.camPos);
		// 	decodePackets[i] = new ReusablePacket(0); // Pre-allocate decode packets
		// 	cam.init();
		// }
		// sense();
	}

	public void sense() {
		// for (int i = 0; i < nCams; i++) {
		// 	VisionHW cam = cameras[i];
		// 	VisionInputsAutoLogged input = inputs[i];

		// 	cam.sense(input);
		// 	Logger.processInputs("/Vision/" + cam.name, input);

		// 	decodePackets[i].setData(input.data);

		// 	List<PhotonPipelineResult> results = decodePackets[i].decodeList(PhotonPipelineResult.photonStruct);

		// 	poseEsts[i].addHeadingData(Timer.getTimestamp(), drive.pose.getRotation());

		// 	for (int j = 0; j < results.size(); j++) {
		// 		PhotonPipelineResult result = results.get(j);

		// 		Optional<EstimatedRobotPose> estPose = poseEsts[i].estimateConstrainedSolvepnpPose(
		// 				result,
		// 				cam.cam.getCameraMatrix().orElseThrow(),
		// 				cam.cam.getDistCoeffs().orElseThrow(),
		// 				new Pose3d(drive.pose),
		// 				false,
		// 				1);
		// 		if (estPose.isPresent()) {
		// 			drive.poseEst.addVisionMeasurement(
		// 					estPose.get().estimatedPose.toPose2d(),
		// 					result.getTimestampSeconds(),
		// 					VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
		// 			continue;
		// 		}

		// 		estPose = poseEsts[i].estimateCoprocMultiTagPose(result);
		// 		if (estPose.isPresent() && result.targets.size() >= 2) {
		// 			drive.poseEst.addVisionMeasurement(
		// 					estPose.get().estimatedPose.toPose2d(),
		// 					result.getTimestampSeconds(),
		// 					VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
		// 			continue;
		// 		}

		// 		estPose = poseEsts[i].estimatePnpDistanceTrigSolvePose(result);
		// 		if (estPose.isPresent()) {
		// 			drive.poseEst.addVisionMeasurement(
		// 					estPose.get().estimatedPose.toPose2d(),
		// 					result.getTimestampSeconds(),
		// 					VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
		// 			continue;
		// 		}
		// 	}
		// }
	}
}
