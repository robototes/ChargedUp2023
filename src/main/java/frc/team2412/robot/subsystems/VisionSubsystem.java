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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
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
	public static final Transform3d ROBOT_TO_CAM =
			IS_COMP
					? new Transform3d(
							new Translation3d(
									// 6.5 inches from back of robot, back is -half of length (30 in.)
									Units.inchesToMeters(-30.0 / 2 + 6.5),
									// 7 inches from left, left is +half of width (26 in.)
									Units.inchesToMeters(26.0 / 2 - 7),
									// 29 inches above the ground
									Units.inchesToMeters(29)),
							// Camera has a slight yaw, -6.7 degrees following right-hand rule (thumb points to
							// +Z/up, fingers curl in positive rotation (CCW looking down))
							new Rotation3d(0, 0, Math.toRadians(-6.7)))
					: new Transform3d(
							new Translation3d(0, 0, Units.inchesToMeters(28)),
							// Camera's upside down
							new Rotation3d(Math.toRadians(180), 0, 0));

	/*
	Test data:
	1
	Physical: 80cm back, 0 left/right
	PV: 0.79 m, 0.08 m, -177.10°

	2
	Physical: 150cm back, 2cm right
	PV: 1.48 m, 0.12 m, -179.03°

	3
	Physical: 202cm back, 3cm right
	PV: 1.98 m, 0.11 m, 175.24°

	4
	Physical 201cm back, 0cm left/right, 23 degrees CCW
	1.86 m	-0.69 m	153.29°

	5
	Physical: 201cm back, 1cm right, 23 degrees CW
	PV: 1.84 m, 0.85 m, -162.76°

	6
	Physical: 201cm back, 44 cm left, 23 degree CW
	PV: 2.00 m, 0.45 m, -156.56°

	7
	Physical: 201cm back, 44 cm left, 0 rotation
	PV: 2.00 m, -0.34 m, -179.24°

	8
	Physical: 148cm back, 43cm left, 0 rotation
	PV: 1.45 m, -0.37 m, 179.29°

	9
	Physical: 148cm back, 43 cm, 23 degree CW
	PV: 1.52 m, 0.20 m, -157.96°

	10
	Physical: 100cm back, 41cm left, 0 rot
	PV: 0.98 m, -0.40 m, -179.83°

	11
	Physical: 100cm back, 41cm left, 23 CW
	PV: 1.08 m, 0.01 m, -159.04°

	12
	Physical: 100cm back, 46cm right, 0 CW
	PV: 0.98 m, 0.52 m, 176.56°

	13
	Physical: 100cm back, 46cm right, 23 CCW
	PV: 1.10 m, 0.05 m, 157.16°

	14
	Physical: 150cm back, 47cm right, 0 rot
	PV: 1.48 m, 0.51 m, 178.36°

	15
	Physical: 150cm back, 47cm right, 23 CCW
	PV: 1.52 m, -0.07 m, 160.33°

	16
	Physical: 200cm back, 46cm right, 0 rot
	PV: 1.98 m, 0.49 m, 177.86°

	17
	Physical: 200cm back, 46cm right, 23 CCW
	PV: 1.99 m, -0.31 m, 161.76°
	*/

	private PhotonCamera photonCamera;
	private Optional<EstimatedRobotPose> latestPose = Optional.empty();
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

		ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
		visionTab.addBoolean("Has targets", this::hasTargets).withPosition(0, 0).withSize(1, 1);
		visionTab
				.addString("Robot pose", () -> String.valueOf(getRobotPose()))
				.withPosition(1, 0)
				.withSize(8, 1);
		visionTab
				.addDouble("Last timestamp", this::getLastTimestampSeconds)
				.withPosition(0, 1)
				.withSize(1, 1);
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
