package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class ArmDiagnosticCommand extends SequentialCommandGroup {
	public ArmDiagnosticCommand(ArmSubsystem armSubsystem) {
		addCommands(
				new SetArmCommand(armSubsystem, LOW),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, PRESCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, SCORE),
				new SetArmCommand(armSubsystem, MIDDLE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, PRESCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, SCORE),
				new SetArmCommand(armSubsystem, SUBSTATION),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, PRESCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, SCORE),
				new SetArmCommand(armSubsystem, HIGH),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, PRESCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, SCORE));
	}
}
