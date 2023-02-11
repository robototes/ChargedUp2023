package frc.team2412.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class DriverAssist {
	private static final PIDController ASSIST_TRANSLATION_PID = new PIDController(0, 0, 0);
	private static final PIDController ASSIST_ROTATION_PID = new PIDController(0, 0, 0);
	private static final PathConstraints ASSIST_CONSTRAINTS = new PathConstraints(1, 0.8);
	private static final double MAX_ALIGNMENT_DISTANCE_METERS = 1.0;

	// comments are relative to the driver, with numbers increasing from right to left
	private static final Pose2d[] alignmentPoses = {
		new Pose2d(new Translation2d(2.0, 0.5), Rotation2d.fromRotations(180)), // 1st cone scoring area
		new Pose2d(
				new Translation2d(2.0, 1.05), Rotation2d.fromRotations(180)), // 1st cube scoring area
		new Pose2d(new Translation2d(2.0, 1.6), Rotation2d.fromRotations(180)), // 2nd cone scoring area
		new Pose2d(
				new Translation2d(2.0, 2.18), Rotation2d.fromRotations(180)), // 3rd cone scoring area
		new Pose2d(
				new Translation2d(2.0, 2.75), Rotation2d.fromRotations(180)), // 2nd cube scoring area
		new Pose2d(new Translation2d(2.0, 3.3), Rotation2d.fromRotations(180)), // 4th cone scoring area
		new Pose2d(
				new Translation2d(2.0, 3.86), Rotation2d.fromRotations(180)), // 5th cone scoring area
		new Pose2d(
				new Translation2d(2.0, 4.43), Rotation2d.fromRotations(180)), // 3rd cube scoring area
		new Pose2d(
				new Translation2d(2.0, 4.98), Rotation2d.fromRotations(180)), // 6th cone scoring area
	};

	private static Pose2d getClosestAlignmentPose(Pose2d robotPose) {
		Translation2d robotTranslation = robotPose.getTranslation();
		double distance = robotTranslation.getDistance(alignmentPoses[0].getTranslation());
		int closestIndex = 0;
		for (int i = 1; i < alignmentPoses.length; i++) {
			double newDistance = robotTranslation.getDistance(alignmentPoses[i].getTranslation());
			if (newDistance > distance) {
				closestIndex = i;
				distance = newDistance;
			}
		}
		return alignmentPoses[closestIndex];
	}

	private static boolean isPoseTooFar(Pose2d robotPose, Pose2d alignmentPose) {
		if (robotPose.getTranslation().getDistance(alignmentPose.getTranslation())
				> MAX_ALIGNMENT_DISTANCE_METERS) {
			return true;
		} else {
			return false;
		}
	}

	public static boolean alignRobot(DrivebaseSubsystem drivebaseSubsystem) {
		Pose2d currentPose = drivebaseSubsystem.getPose();
		Pose2d alignmentPose = getClosestAlignmentPose(currentPose);
		if (isPoseTooFar(currentPose, alignmentPose)) {
			return false;
		}

		PathPlannerTrajectory alignmentTraj =
				PathPlanner.generatePath(
						ASSIST_CONSTRAINTS,
						new PathPoint(currentPose.getTranslation(), currentPose.getRotation()),
						new PathPoint(alignmentPose.getTranslation(), alignmentPose.getRotation()));

		Command followAlignmentCommand =
				new PPSwerveControllerCommand(
						alignmentTraj,
						drivebaseSubsystem::getPose,
						DrivebaseSubsystem.kinematics,
						ASSIST_TRANSLATION_PID,
						ASSIST_TRANSLATION_PID,
						ASSIST_ROTATION_PID,
						drivebaseSubsystem::drive,
						true,
						drivebaseSubsystem);

		followAlignmentCommand.schedule();
		return true;
	}
}
