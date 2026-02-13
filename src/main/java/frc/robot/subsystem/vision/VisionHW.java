package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.ReusablePacket;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionHW {
	public PhotonCamera cam;
	public String name;
	public Transform3d camPos;

	public ReusablePacket encodePacket = new ReusablePacket(1024);

	public VisionHW(String name, Transform3d camPos) {
		this.name = name;
		this.camPos = camPos;
	}

	public void init() {
		if (!Robot.isReal()) return;

		cam = new PhotonCamera(name);
	}

	public void sense(VisionInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.isConnected = cam.isConnected();

		List<PhotonPipelineResult> res = cam.getAllUnreadResults();

		encodePacket.reset();
		encodePacket.encodeList(res);

		inputs.data = encodePacket.getDataReference();
	}
}
