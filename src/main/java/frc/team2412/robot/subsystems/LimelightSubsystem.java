package frc.team2412.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
	public static final double CAMERA_HEIGHT = 0.2;
	public static final double CAMERA_ANGLE_OFFSET = 0;

	public static final double GOAL_DISTANCE_FROM_TARGET = 0.3;
	public static final double GOAL_DISTANCE_FROM_CONE = 0.3;
	public static final double GOAL_DISTANCE_FROM_CUBE = 0.3;

	// MEMBERS

	NetworkTable networkTable;
	String currentPoseString;
	String targetPoseString;

	// network tables

	// CONSTRUCTOR !
	public LimelightSubsystem() {

		// logging
		currentPoseString = "";
		targetPoseString = "";

		ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
		networkTable = NetworkTableInstance.getDefault().getTable("limelight");

		limelightTab.addBoolean("hasTarget", this::hasTargets).withPosition(1, 0).withSize(1, 1);
		limelightTab
				.addDouble("Horizontal Offset", this::getHorizontalOffset)
				.withPosition(2, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Vertical Offset", this::getVerticalOffset)
				.withPosition(2, 0)
				.withSize(1, 1);

		limelightTab
				.addDouble("Target Distance ", this::getDistanceFromTarget)
				.withPosition(4, 0)
				.withSize(1, 1);

		limelightTab
				.addString("Current Pose ", this::getCurrentPoseString)
				.withPosition(3, 0)
				.withSize(1, 1);

		limelightTab
				.addString("Target Pose ", this::getTargetPoseString)
				.withPosition(4, 0)
				.withSize(1, 1);
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

	public double getDistanceFromTarget() {
		double targetHeight = 0;
		double angleToTarget = 0;
		return (targetHeight - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE_OFFSET + angleToTarget);
	}

	public Pose2d getTargetPose(Pose2d currentPose) {

		// math thing to get target pose using current pose

		Rotation2d currentHeading = currentPose.getRotation();
		Rotation2d targetHeading = new Rotation2d(currentHeading.getDegrees() + getHorizontalOffset());

		double targetX = Math.sin(targetHeading.getDegrees()) * getDistanceFromTarget();
		double targetY = Math.cos(targetHeading.getDegrees()) * getDistanceFromTarget();

		Pose2d targetPose = new Pose2d(targetX, targetY, targetHeading);

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

		System.out.println("current: " + drivebaseSubsystem.getPose());
		System.out.println("target: " + getTargetPose(drivebaseSubsystem.getPose()));
		System.out.println("distance: " + getDistanceFromTarget());
		System.out.println("horizontal offset: " + getHorizontalOffset());
		System.out.println("vertical offset; " + getVerticalOffset());

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
	public void periodic() {
		System.out.println("has target " + hasTargets());
		System.out.println("distance " + getDistanceFromTarget());
	}
}
