package frc.team2412.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LimelightSubsystem;

public class GetWithinDistanceCommand extends CommandBase {

	private LimelightSubsystem limelightSubsystem;
	private DrivebaseSubsystem drivebaseSubsystem;

	public GetWithinDistanceCommand(
			LimelightSubsystem limelightSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
		this.limelightSubsystem = limelightSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
	}

	public void execute() {
		// TOOD: use driver assist code
		// get drivebase pose -> feed into get pose method -> use driver assist code

		Pose2d currentPose = drivebaseSubsystem.getPose();

		// driver assist code

		limelightSubsystem.getWithinDistance(currentPose);
	}
}
