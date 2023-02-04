package frc.team2412.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import io.github.oblarg.oblog.annotations.Log;
import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * All poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 *
 * <p>At some point we should discuss how we want to handle the different alliances.
 */
public class VisionSubsystem extends SubsystemBase {
	private PhotonCamera photonCamera;
	private PhotonPipelineResult latestResult;
	private BiConsumer<Pose2d, Double> poseConsumer;
	/** Null if no known robot pose, otherwise the last calculated robot pose from vision data. */
	private Pose3d robotPose = null;
	private double lastTimeStamp=0;
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
		} catch (IOException err) {
			DriverStation.reportError("Could not load AprilTagFieldLayout! " + err, err.getStackTrace());
			temp = null;
		}
		fieldLayout = temp;
	}

	public VisionSubsystem(BiConsumer<Pose2d, Double> poseConsumer) {
		this.poseConsumer = poseConsumer;

		var networkTables = NetworkTableInstance.getDefault();

		// Connect to photonvision server
		// Only in sim because normally photonvision connects to robot
		if (RobotBase.isSimulation()) {
			//			networkTables.stopServer();
			//			networkTables.startClient4("localhost");

		}

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		latestResult = photonCamera.getLatestResult();

		networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(Hardware.PHOTON_CAM)
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				this::updateEvent);
	}

	public void updateEvent(NetworkTableEvent event) {
		latestResult = photonCamera.getLatestResult();
		updatePoseCache();
		if (robotPose != null) {
			lastTimeStamp = latestResult.getTimestampSeconds();
			poseConsumer.accept(robotPose.toPose2d(), lastTimeStamp);
		}
	}

	@Log
	public boolean hasTargets() {
		return latestResult.hasTargets();
	}

	/**
	 * Calculates the robot pose relative to the field, using the given target.
	 *
	 * <p>Returns null if target is null or if no valid field pose could be found for the target
	 * (either the target doesn't have an ID or we don't have a pose associated with the ID).
	 *
	 * @param target The target to use to calculate robot pose.
	 * @return The calculated robot pose.
	 */
	public static Pose3d getRobotPoseUsingTarget(PhotonTrackedTarget target) {
		if (target == null || fieldLayout == null) {
			return null;
		}
		// If target doesn't have fiducial ID, value is -1 (which shouldn't be in the layout)
		Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
		if (tagPose.isEmpty()) {
			return null;
		}
		// getBestCameraToTarget() shouldn't be null
		return tagPose
				.get()
				.transformBy(target.getBestCameraToTarget().inverse())
				.transformBy(Hardware.CAM_TO_ROBOT);
	}

	public static Pose3d getAlternateRobotPoseUsingTarget(PhotonTrackedTarget target) {
		if (target == null || fieldLayout == null) {
			return null;
		}
		// If target doesn't have fiducial ID, value is -1 (which shouldn't be in the layout)
		Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
		if (tagPose.isEmpty()) {
			return null;
		}
		return tagPose
				.get()
				.transformBy(target.getAlternateCameraToTarget().inverse())
				.transformBy(Hardware.CAM_TO_ROBOT);
	}

	/** Updates the robot pose cache. */
	private void updatePoseCache() {
		if (!hasTargets()) {
			robotPose = null;
		} else {
			robotPose = getRobotPoseUsingTarget(latestResult.getBestTarget());
		}
		System.out.println(
				"Best: " + latestResult.getBestTarget().getBestCameraToTarget().getRotation().getX());
		System.out.println(
				"Alternate: "
						+ latestResult.getBestTarget().getAlternateCameraToTarget().getRotation().getX());
	}

	/**
	 * Calculates the robot pose using the best target. Returns null if there is no known robot pose.
	 *
	 * @return The calculated robot pose in meters.
	 */
	public Pose3d getRobotPose() {
		return robotPose;
	}

	//returns the last time we saw an aprilTag
	public double getLastTimeStamp(){
		return lastTimeStamp;
	}
}
