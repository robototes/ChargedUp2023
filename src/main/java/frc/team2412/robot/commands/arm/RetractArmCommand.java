package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.RETRACT_ARM_ANGLE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.RETRACT_WRIST_ANGLE;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class RetractArmCommand extends ParallelCommandGroup {

	public RetractArmCommand(ArmSubsystem armSubsystem) {
		addCommands(
				new SetArmCommand(armSubsystem, RETRACT_ARM_ANGLE),
				new SetWristCommand(armSubsystem, RETRACT_WRIST_ANGLE));
	}
}
