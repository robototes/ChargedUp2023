package frc.team2412.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

	// CONSTANTS

	// meters?
	public static final double CAMERA_HEIGHT = 0.2;
	public static final double CAMERA_ANGLE_OFFSET = 0;

	public static final double GOAL_DISTANCE_FROM_TARGET = 0.3;
	public static final double GOAL_DISTANCE_FROM_CONE = 0.3;
	public static final double GOAL_DISTANCE_FROM_CUBE = 0.3;

	// MEMBERS

	NetworkTable networkTable;

	// network tables

	// CONSTRUCTOR !
	public LimelightSubsystem() {

		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
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

		return currentPose; // replace later
	}

	public void getWithinDistance(Pose2d currentPose) {
		getTargetPose(currentPose);
	}
}
