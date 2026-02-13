package frc.robot.subsystem.vision;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
	public VisionHW[] hws = new VisionHW[] {new VisionHW("TODO", new Transform3d())};
	public VisionInputsAutoLogged[] inputs = new VisionInputsAutoLogged[hws.length];
	public PhotonPoseEstimator[] poseEst = new PhotonPoseEstimator[hws.length];

	public static AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public int n = hws.length;

	public void init() {
		for (int i = 0; i < n; i++) {
			VisionHW hw = hws[i];
			inputs[i] = new VisionInputsAutoLogged();
			poseEst[i] = new PhotonPoseEstimator(tagLayout, hw.camPos);
			hw.init();
		}
		sense();
	}

	public void sense() {
		for (int i = 0; i < n; i++) {
			VisionHW hw = hws[i];
			VisionInputsAutoLogged input = inputs[i];

			hw.sense(input);
			Logger.processInputs("/Vision/" + hw.name, input);

			Packet dat = new Packet(input.data);
			List<PhotonPipelineResult> res = dat.decodeList(PhotonPipelineResult.photonStruct);

			poseEst[i].addHeadingData(Timer.getTimestamp(), drive.pose.getRotation());

			for (int j = 0; j < res.size(); j++) {
				Optional<EstimatedRobotPose> camPose = poseEst[i].estimateConstrainedSolvepnpPose(
						res.get(j),
						hw.cam.getCameraMatrix().orElseThrow(),
						hw.cam.getDistCoeffs().orElseThrow(),
						new Pose3d(drive.pose),
						false,
						1);
				if (!camPose.isEmpty()) {
					drive.poseEst.addVisionMeasurement(
							camPose.get().estimatedPose.toPose2d(),
							res.get(j).getTimestampSeconds(),
							VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
					continue;
				}

				camPose = poseEst[i].estimateCoprocMultiTagPose(res.get(j));
				if (!camPose.isEmpty() && res.get(j).targets.size() >= 2) {
					drive.poseEst.addVisionMeasurement(
							camPose.get().estimatedPose.toPose2d(),
							res.get(j).getTimestampSeconds(),
							VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
					continue;
				}

				camPose = poseEst[i].estimatePnpDistanceTrigSolvePose(res.get(j));
				if (!camPose.isEmpty()) {
					drive.poseEst.addVisionMeasurement(
							camPose.get().estimatedPose.toPose2d(),
							res.get(j).getTimestampSeconds(),
							VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)
					continue;
				}
			}
		}
	}
}
