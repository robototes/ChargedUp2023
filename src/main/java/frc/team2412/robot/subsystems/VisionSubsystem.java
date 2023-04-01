package frc.team2412.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.team2412.robot.util.ShuffleboardUtil;
import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * All 3D poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 *
 * <p>2D field poses are different. +X is away from the driver and +Y is toward the opposing loading
 * station. Rotations are CCW+ looking down. When on the blue alliance, this means that from the
 * (blue) driver's perspective +X is away and +Y is to the left. When on the red alliance, this
 * means that from the (red) driver's perspective +X is away and +Y is to the right.
 */
public class VisionSubsystem extends SubsystemBase {
	private static final boolean IS_COMP = Robot.getInstance().isCompetition();
	// Rough measurements, origin is center of robot, +X is forward, +Y is left, +Z is up
	public static final Transform3d ROBOT_TO_CAM =
			IS_COMP
					? new Transform3d(
							new Translation3d(
									// 6 5/8 inches from back of robot, back is -half of length (30 in.)
									Units.inchesToMeters(-30.0 / 2 + 6.625),
									// 6 3/4 inches from left, left is +half of width (26 in.)
									Units.inchesToMeters(26.0 / 2 - 6.75),
									// 39 1/4 inches above the ground
									Units.inchesToMeters(39.25)),
							// Limelight top back edge is 1 inch forward from bottom back edge (opposite), and
							// limelight is 2 3/16 inches tall (hypotenuse)
							// Pitch is positive following right-hand rule (thumb points to +Y/left, fingers curl
							// positive rotation (CCW looking right))
							new Rotation3d(0, Math.asin(1 / 2.1875), 0))
					: new Transform3d(
							new Translation3d(
									// 4 3/4 inches from front of robot, front is +half of length (24 in.)
									Units.inchesToMeters(24.0 / 2 - 4.75),
									// 7 1/8 inches from left of robot, left is +half of width (24 in.)
									Units.inchesToMeters(24.0 / 2 - 7.125),
									// 41.5 inches above the ground
									Units.inchesToMeters(41.5)),
							new Rotation3d(0, 0, 0));
	public static final double MAX_TRUSTABLE_HORIZONTAL_DISTANCE = 3;
	// This is from the metric approximations from section 5.1 of the game manual
	private static final double FIELD_LENGTH_METERS = 16.54;

	private static Pose2d convertToFieldPose(Pose3d pose3d) {
		return convertToFieldPose(pose3d, DriverStation.getAlliance());
	}

	private static Pose2d convertToFieldPose(Pose3d pose3d, DriverStation.Alliance alliance) {
		switch (alliance) {
			case Red:
				return new Pose2d(
						FIELD_LENGTH_METERS - pose3d.getX(),
						pose3d.getY(),
						new Rotation2d(pose3d.getRotation().getZ()));
			case Invalid:
				DriverStation.reportWarning("Unknown alliance! Assuming blue", true);
				// fall through
			default:
				return pose3d.toPose2d();
		}
	}

	private final PhotonCamera photonCamera;
	private Optional<EstimatedRobotPose> latestPose = Optional.empty();
	private final PhotonPoseEstimator photonPoseEstimator;
	private final SwerveDrivePoseEstimator poseEstimator;
	private boolean targetTooFar = false;

	private double lastTimestampSeconds = 0;
	private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());

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
		synchronized (poseEstimator) {
			poseEstimator.setVisionMeasurementStdDevs(
					VecBuilder.fill(0.0385, 0.0392, Math.toRadians(2.85)));
		}

		var networkTables = NetworkTableInstance.getDefault();
		if (Robot.isSimulation()) {
			networkTables.stopServer();
			networkTables.startClient4("localhost");
		}

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		this.photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, ROBOT_TO_CAM);

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
				.addDouble("Last timestamp", this::getLastTimestampSeconds)
				.withPosition(1, 0)
				.withSize(1, 1);
		ShuffleboardUtil.addPose3dLayout(visionTab, "Robot pose", this::getRobotPose, 2, 0);
		visionTab
				.addBoolean("Target was too far", () -> targetTooFar)
				.withPosition(0, 1)
				.withSize(1, 1);
	}

	public void updateEvent(NetworkTableEvent event) {
		PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
		targetTooFar =
				pipelineResult.getTargets().stream()
						.noneMatch(
								(target) ->
										target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
												< MAX_TRUSTABLE_HORIZONTAL_DISTANCE);
		latestPose = photonPoseEstimator.update(pipelineResult);
		if (latestPose.isPresent()) {
			lastTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = convertToFieldPose(latestPose.get().estimatedPose);
			if (!targetTooFar) {
				synchronized (poseEstimator) {
					poseEstimator.addVisionMeasurement(lastFieldPose, lastTimestampSeconds);
				}
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
