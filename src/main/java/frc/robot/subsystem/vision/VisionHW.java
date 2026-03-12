package frc.robot.subsystem.vision;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import java.io.File;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.ReusablePacket;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionHW {
	public PhotonCamera cam;
	public String name;
	public Transform3d camPos;
	public String calibFile;

	// Calibration loaded from deploy JSON; used instead of NT values
	public double[] calibCameraMatrix = new double[0];
	public double[] calibDistCoeffs = new double[0];

	public ReusablePacket encodePacket = new ReusablePacket(1024);

	public VisionHW(String name, Transform3d camPos, String calibFile) {
		this.name = name;
		this.camPos = camPos;
		this.calibFile = calibFile;
	}

	public void init() {
		// Load calibration from deploy JSON
		try {
			File f = new File(Filesystem.getDeployDirectory(), calibFile);
			JsonNode root = new ObjectMapper().readTree(f);
			JsonNode intrData = root.get("cameraIntrinsics").get("data");
			JsonNode distData = root.get("distCoeffs").get("data");
			calibCameraMatrix = new double[intrData.size()];
			for (int i = 0; i < intrData.size(); i++)
				calibCameraMatrix[i] = intrData.get(i).asDouble();
			calibDistCoeffs = new double[distData.size()];
			for (int i = 0; i < distData.size(); i++)
				calibDistCoeffs[i] = distData.get(i).asDouble();
			System.out.println("Loaded calibration for " + name + " from " + calibFile);
		} catch (Exception e) {
			System.out.println("Warning: failed to load calibration for " + name + ": " + e.getMessage());
		}

		if (!Robot.isReal()) return;

		cam = new PhotonCamera(name);
	}

	public void sense(VisionInputs inputs) {
		// Always use file-loaded calibration
		inputs.cameraMatrix = calibCameraMatrix;
		inputs.distCoeffs = calibDistCoeffs;

		if (!Robot.isReal()) return;

		inputs.isConnected = cam.isConnected();

		List<PhotonPipelineResult> results = cam.getAllUnreadResults();

		encodePacket.reset();
		encodePacket.encodeList(results);

		inputs.data = encodePacket.getDataReference();
	}
}
