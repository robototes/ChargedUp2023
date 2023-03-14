package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.arm.SetWristCommand.WristPosition;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class SetFullArmCommand extends SequentialCommandGroup {
	public SetFullArmCommand(
			ArmSubsystem armSubsystem,
			IntakeSubsystem intakeSubsystem,
			PositionType positionType,
			WristPosition wristPosition) {
		addCommands(
				new SetArmCommand(armSubsystem, positionType),
				new SetWristCommand(armSubsystem, intakeSubsystem, wristPosition));
	}
}
