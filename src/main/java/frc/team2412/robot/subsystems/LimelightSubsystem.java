package frc.team2412.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

	// CONSTANTS

	private static final PIDController TRANSLATION_PID = new PIDController(10.0, 0, 0);
	private static final PIDController ROTATION_PID = new PIDController(8.0, 0, 0);
	PathConstraints PATH_CONSTRAINTS = new PathConstraints(4.0, 2.0);

	// meters?
	public static final double CAMERA_MOUNT_HEIGHT = 0.1143;
	public static final double CAMERA_ANGLE_OFFSET = 0;
	public static final double TARGET_HEIGHT = 0.36;

	public static final double GOAL_DISTANCE_FROM_TARGET = 0.7;
	public static final double GOAL_DISTANCE_FROM_CONE = 0.3;
	public static final double GOAL_DISTANCE_FROM_CUBE = 0.3;

	// MEMBERS

	NetworkTable networkTable;

	String currentPoseString;
	String targetPoseString;

	// network tables

	// CONSTRUCTOR !
	public LimelightSubsystem() {

		// broadcast 10.24.12.227

		// logging
		currentPoseString = "";
		targetPoseString = "";

		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
		ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

		limelightTab.addBoolean("hasTarget", this::hasTargets).withPosition(0, 0).withSize(1, 1);
		limelightTab
				.addDouble("Horizontal Offset", this::getHorizontalOffset)
				.withPosition(1, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Vertical Offset", this::getVerticalOffset)
				.withPosition(2, 0)
				.withSize(1, 1);

		limelightTab
				.addDouble("Target Distance ", this::getDistanceFromTarget)
				.withPosition(3, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Target Distance 2 - TEST ", this::getDistanceFromTargetTheSecond)
				.withPosition(4, 0)
				.withSize(1, 1);
		limelightTab
				.addString("Current Pose ", this::getCurrentPoseString)
				.withPosition(0, 1)
				.withSize(4, 1);

		limelightTab
				.addString("Target Pose ", this::getTargetPoseString)
				.withPosition(0, 2)
				.withSize(4, 1);
	}

	// METHODS

	public boolean hasTargets() {
		return (networkTable.getEntry("tv").getDouble(0) != 0);
	}

	public double getHorizontalOffset() {
		return networkTable.getEntry("tx").getDouble(0);
	}

	public double getVerticalOffset() {
		return networkTable.getEntry("ty").getDouble(0);
	}

	public double getBoxWidth() {
		return networkTable.getEntry("tshort").getDouble(0);
	}

	public double getDistanceFromTarget() {

		// 1.3 at 1.9
		// 1.0 at 1.3

		double angleToTarget = getVerticalOffset();
		// return (TARGET_HEIGHT - CAMERA_MOUNT_HEIGHT) / Math.tan(CAMERA_ANGLE_OFFSET + angleToTarget);

		return (TARGET_HEIGHT - CAMERA_MOUNT_HEIGHT)
				/ (CAMERA_ANGLE_OFFSET + Math.tan(Units.degreesToRadians(getVerticalOffset())));
	}

	public double getDistanceFromTargetTheSecond() {

		// focal length = (P x D) / W
		double focal_length = 405.705882353;

		// distance = (W x F) / P
		// returns inches for testing purposes, will divide by 39.3700787 to return meters
		return (8.5 * focal_length) / getBoxWidth();
	}

	// tan(degree) * distance = sideways distance

	// target height / tan(vertical angle)

	public Pose2d getTargetPose(Pose2d currentPose) {

		// math thing to get target pose using current pose

		// FIXME: figure out why targetPose returns infinity

		Rotation2d currentHeading = currentPose.getRotation();
		Rotation2d targetHeading =
				new Rotation2d(
						currentPose.getRotation().getRadians() + Units.degreesToRadians(getHorizontalOffset()));
		double targetDistance = getDistanceFromTarget() - GOAL_DISTANCE_FROM_TARGET;

		double targetX = Math.sin(targetHeading.getRadians()) * targetDistance;
		double targetY = Math.cos(targetHeading.getRadians()) * targetDistance;

		Pose2d targetPose =
				new Pose2d(currentPose.getX() + targetY, currentPose.getY() + targetX, targetHeading);

		// target pose always ~7.61 degrees?

		currentPoseString = currentPose.toString();
		targetPoseString = targetPose.toString();

		return targetPose;
	}

	public String getCurrentPoseString() {
		return currentPoseString;
	}

	public String getTargetPoseString() {
		return targetPoseString;
	}

	public boolean isWithinDistance() {
		return (getDistanceFromTarget() <= GOAL_DISTANCE_FROM_TARGET);
	}

	public Command getWithinDistance(Pose2d currentPose, DrivebaseSubsystem drivebaseSubsystem) {
		Pose2d targetPose;
		if (hasTargets()) {
			targetPose = getTargetPose(currentPose);

		} else {
			targetPose = currentPose;
		}

		// create path

		PathPlannerTrajectory alignmentTraj =
				PathPlanner.generatePath(
						PATH_CONSTRAINTS,
						new PathPoint(
								currentPose.getTranslation(),
								Rotation2d.fromRadians(
										Math.atan2(
												targetPose.getY() - currentPose.getY(),
												targetPose.getX() - currentPose.getX())),
								currentPose.getRotation(),
								drivebaseSubsystem.getVelocity()),
						new PathPoint(
								targetPose.getTranslation(),
								Rotation2d.fromDegrees(180),
								targetPose.getRotation()));

		System.out.println(alignmentTraj);
		// make command out of path

		Command moveCommand =
				new PPSwerveControllerCommand(
						alignmentTraj,
						drivebaseSubsystem::getPose,
						DrivebaseSubsystem.kinematics,
						TRANSLATION_PID,
						TRANSLATION_PID,
						ROTATION_PID,
						drivebaseSubsystem::drive,
						true,
						drivebaseSubsystem);

		return moveCommand;
	}

	@Override
	public void periodic() {}
}
