package frc.team2412.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

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
	private static final double MAX_TRUSTABLE_XY_DISTANCE = 2;
	private static final double MAX_TRUSTABLE_AMBIGUITY = 0.1;
	private static final double EDGE_THRESHOLD_PIXELS = 10;
	private static final double RESOLUTION_WIDTH = 960;
	private static final double RESOLUTION_HEIGHT = 720;
	// These are from the metric approximations from section 5.1 of the game manual
	private static final double FIELD_LENGTH_METERS = 16.54;
	private static final double FIELD_WIDTH_METERS = 8.02;

	private static Pose2d convertToFieldPose(Pose3d pose3d, DriverStation.Alliance alliance) {
		switch (alliance) {
			case Red:
				return new Pose2d(
						// FIELD_LENGTH_METERS - pose3d.getX(),
						pose3d.getX(),
						FIELD_WIDTH_METERS - pose3d.getY(),
						new Rotation2d(2 * Math.PI + pose3d.getRotation().getZ()));
			case Invalid:
				DriverStation.reportWarning("Unknown alliance! Assuming blue", true);
				// fall through
			default:
				return pose3d.toPose2d();
		}
	}

	private final PhotonCamera photonCamera;
	private final PhotonPoseEstimator photonPoseEstimator;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final FieldObject2d visionOnlyPoseObject;
	private DriverStation.Alliance alliance;
	// These are always set with every pipeline result
	private PhotonPipelineResult latestResult = null;
	private Optional<EstimatedRobotPose> latestPose = Optional.empty();
	private boolean targetTooFar = false;
	private boolean ambiguityTooHigh = false;
	private boolean tooCloseToEdge = false;

	// These are only set when there's a valid pose
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

	public VisionSubsystem(SwerveDrivePoseEstimator initialPoseEstimator, Field2d field) {
		poseEstimator = initialPoseEstimator;
		visionOnlyPoseObject = field.getObject("VisionPose");

		var networkTables = NetworkTableInstance.getDefault();
		if (Robot.isSimulation()) {
			networkTables.stopServer();
			networkTables.startClient4("localhost");
		}

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		this.photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, ROBOT_TO_CAM);

		alliance = DriverStation.getAlliance();

		networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(Hardware.PHOTON_CAM)
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				event -> update());

		ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
		visionTab.addBoolean("Has targets", this::hasTargets).withPosition(0, 0).withSize(1, 1);
		visionTab.addInteger("Num targets", this::getNumTargets).withPosition(0, 1).withSize(1, 1);
		visionTab
				.addDouble("Last timestamp", this::getLastTimestampSeconds)
				.withPosition(1, 0)
				.withSize(1, 1);
		visionTab
				.addBoolean("Target was too far", () -> targetTooFar)
				.withPosition(1, 1)
				.withSize(1, 1);
		visionTab
				.addBoolean("Too high ambiguity", () -> ambiguityTooHigh)
				.withPosition(1, 2)
				.withSize(1, 1);
		visionTab
				.addBoolean("Corner too close to edge", () -> tooCloseToEdge)
				.withPosition(1, 3)
				.withSize(1, 1);
		ShuffleboardUtil.addPose3dLayout(visionTab, "Robot pose", this::getRobotPose, 2, 0);
	}

	/**
	 * Calculates the standard deviations of the vision measurements. Also updates some logging
	 * variables.
	 *
	 * @param result The current pipeline result.
	 * @return The standard deviations of the vision measurements. Null is used to indicate the
	 *     measurement should not be added.
	 */
	private Vector<N3> getStdDevs(PhotonPipelineResult result) {
		Vector<N3> stdDevs = VecBuilder.fill(0.0385, 0.0392, Math.toRadians(2.85));
		double minTargetDistance = Double.POSITIVE_INFINITY;
		double minTargetAmbiguity = Double.POSITIVE_INFINITY;
		tooCloseToEdge = false;
		for (PhotonTrackedTarget target : result.getTargets()) {
			double xyDistance =
					target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
			if (xyDistance < minTargetDistance) {
				minTargetDistance = xyDistance;
			}
			if (target.getPoseAmbiguity() < minTargetAmbiguity) {
				minTargetAmbiguity = target.getPoseAmbiguity();
			}
			for (TargetCorner corner : target.getDetectedCorners()) {
				tooCloseToEdge |=
						(corner.x <= EDGE_THRESHOLD_PIXELS
								|| corner.x >= RESOLUTION_WIDTH - EDGE_THRESHOLD_PIXELS
								|| corner.y <= EDGE_THRESHOLD_PIXELS
								|| corner.y >= RESOLUTION_HEIGHT - EDGE_THRESHOLD_PIXELS);
			}
		}
		targetTooFar = minTargetDistance >= MAX_TRUSTABLE_XY_DISTANCE;
		ambiguityTooHigh = minTargetAmbiguity >= MAX_TRUSTABLE_AMBIGUITY;
		if (targetTooFar || ambiguityTooHigh || tooCloseToEdge) {
			return null;
		}
		return stdDevs;
	}

	public void update() {
		PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
		latestResult = pipelineResult;
		latestPose = photonPoseEstimator.update(pipelineResult);
		if (latestPose.isPresent()) {
			lastTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = convertToFieldPose(latestPose.get().estimatedPose, alliance);
			visionOnlyPoseObject.setPose(lastFieldPose);
			Vector<N3> stdDevs = getStdDevs(pipelineResult);
			if (stdDevs != null) {
				synchronized (poseEstimator) {
					poseEstimator.addVisionMeasurement(lastFieldPose, lastTimestampSeconds, stdDevs);
				}
			}
		}
	}

	public boolean hasTargets() {
		return latestPose.isPresent();
	}

	public int getNumTargets() {
		return latestResult == null ? -1 : latestResult.getTargets().size();
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

	public void setAlliance(DriverStation.Alliance alliance) {
		this.alliance = alliance;
	}
}
