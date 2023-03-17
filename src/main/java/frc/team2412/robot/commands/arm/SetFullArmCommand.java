package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;

public class SetFullArmCommand extends SequentialCommandGroup {
	public SetFullArmCommand(ArmSubsystem armSubsystem, PositionType positionType) {
		addCommands(
				new SetArmCommand(armSubsystem, positionType),
				new SetWristCommand(armSubsystem, WRIST_PRESCORE));
	}
}
