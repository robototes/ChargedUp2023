package frc.team2412.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import io.github.oblarg.oblog.annotations.Log;
import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
	private PhotonCamera photonCamera;
	private PhotonPipelineResult latResult;
	/** Null if invalid, Empty if no valid camera pose, otherwise contains the camera pose. */
	private Optional<Pose3d> cameraPose = null;
	/*
	 * Because we have to handle an IOException, we can't initialize fieldLayout in the variable declaration (private static final AprilTagFieldLayout fieldLayout = ...;). Instead, we have to initialize it in a static initializer (static { ... }).
	 */
	private static final AprilTagFieldLayout fieldLayout;

	static {
		/*
		 * This code runs when the class is initialized (same time as normal variable initializers).
		 *
		 * We have to use temp so that the compiler knows we initialize fieldLayout exactly once. If we directly initialize it, the compiler thinks we might initialize it twice if we get an IOException after initializing it in the try block (since then we'd run the catch block and initialize there too).
		 */
		AprilTagFieldLayout temp;

		try {
			temp = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
		} catch (IOException e) {
			e.printStackTrace();
			temp = null;
		}
		fieldLayout = temp;
	}

	public VisionSubsystem() {
		var instance = NetworkTableInstance.getDefault();

		// Connect to photonvision server
		// Only in sim because normally photonvision connects to robot
		if (RobotBase.isSimulation()) {
			instance.stopServer();
			instance.startClient4("localhost");
		}

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		latResult = photonCamera.getLatestResult();
		instance.addListener(
				instance.getTable("photonvision").getSubTable(Hardware.PHOTON_CAM).getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				(notif) -> {
					latResult = photonCamera.getLatestResult();
					cameraPose = null; // New data, invalidate camera pose
				});
	}

	@Log
	public boolean hasTargets() {
		return latResult.hasTargets();
	}

	/**
	 * Calculates the camera pose relative to the field, using the given target.
	 *
	 * <p>Returns null if target is null or if no valid field pose could be found for the target
	 * (either the target doesn't have an ID or we don't have a pose associated with the ID).
	 *
	 * @param target The target to use to calculate camera pose.
	 * @return The calculated camera pose.
	 */
	public static Pose3d getCameraPoseUsingTarget(PhotonTrackedTarget target) {
		if (target == null) {
			return null;
		}
		// If target doesn't have fiducial ID, value is -1 (which shouldn't be in the layout)
		Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
		if (tagPose.isEmpty()) {
			return null;
		}
		// getBestCameraToTarget() shouldn't be null
		return tagPose.get().transformBy(target.getBestCameraToTarget().inverse());
	}

	/**
	 * Calculates the camera pose using the best target. Returns null if there is no valid pose.
	 *
	 * @return The calculated camera pose.
	 */
	public Pose3d getCameraPose() {
		if (cameraPose == null) {
			// No cached value, calculate current camera pose if possible
			if (!hasTargets()) {
				cameraPose = Optional.empty();
			} else {
				cameraPose = Optional.ofNullable(getCameraPoseUsingTarget(latResult.getBestTarget()));
			}
		}
		// Unwrap Optional, defaulting to null if it's empty
		return cameraPose.orElse(null);
	}
}
