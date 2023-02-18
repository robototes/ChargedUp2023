package frc.team2412.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
import io.github.oblarg.oblog.annotations.Log;
import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * All poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 *
 * <p>At some point we should discuss how we want to handle the different alliances.
 */
public class VisionSubsystem extends SubsystemBase {
	private static final boolean IS_COMP = Robot.getInstance().isCompetition();
	// Rough measurements, origin is center of robot, +X is forward, +Y is left, +Z is up
	// TODO Get final CAD measurements
	public static final Transform3d ROBOT_TO_CAM =
			IS_COMP
					? new Transform3d(
							new Translation3d(
									// 7 inches from back of robot, back is -half of length (30 in.)
									Units.inchesToMeters(-30.0 / 2 + 7),
									// 7 inches from left, left is +half of width (26 in.)
									Units.inchesToMeters(26.0 / 2 - 7),
									// 30 inches above the ground
									Units.inchesToMeters(30)),
							// Camera has a slight yaw, -12 degrees following right-hand rule (thumb points to
							// +Z/up, fingers curl in positive rotation)
							new Rotation3d(0, 0, Math.toRadians(-12)))
					: new Transform3d(
							new Translation3d(
									// 0.5 inches from front of robot, front is +half of length (24 in.)
									Units.inchesToMeters(24 * 0.5 - 0.5),
									// Around 1 inch to the right
									Units.inchesToMeters(1),
									// 26.5 inches above the ground
									Units.inchesToMeters(26.5)),
							// Camera's pointed backwards
							new Rotation3d(0, 0, Math.toRadians(180)));

	private PhotonCamera photonCamera;
	private Optional<EstimatedRobotPose> latestPose;
	private PhotonPoseEstimator photonPoseEstimator;
	private final SwerveDrivePoseEstimator poseEstimator;

	private double lastTimestampSeconds = 0;
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

	public VisionSubsystem(SwerveDrivePoseEstimator initialPoseEstimator) {
		poseEstimator = initialPoseEstimator;

		var networkTables = NetworkTableInstance.getDefault();
		if (Robot.isSimulation()) {
			networkTables.stopServer();
			networkTables.startClient4("localhost");
		}

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		this.photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, ROBOT_TO_CAM);

		networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(Hardware.PHOTON_CAM)
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				this::updateEvent);
	}

	public void updateEvent(NetworkTableEvent event) {
		latestPose = photonPoseEstimator.update();
		if (latestPose.isPresent()) {
			lastTimestampSeconds = latestPose.get().timestampSeconds;
			synchronized (poseEstimator) {
				poseEstimator.addVisionMeasurement(
						latestPose.get().estimatedPose.toPose2d(), lastTimestampSeconds);
			}
		}
	}

	@Log
	public boolean hasTargets() {
		return latestPose.isPresent();
	}

	/**
	 * Calculates the robot pose using the best target. Returns null if there is no known robot pose.
	 *
	 * @return The calculated robot pose in meters.
	 */
	public Pose3d getRobotPose() {
		if (latestPose.isPresent()) {
			return latestPose.get().estimatedPose;
		}
		return null;
	}

	/**
	 * Returns the last time we saw an AprilTag.
	 *
	 * @return The time we last saw an AprilTag in seconds since FPGA startup.
	 */
	public double getLastTimestampSeconds() {
		return lastTimestampSeconds;
	}
}
