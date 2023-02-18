package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class ArmDiagnosticCommand extends SequentialCommandGroup {
	public ArmDiagnosticCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
		addCommands(
				new SetArmCommand(armSubsystem, ARM_LOW_POSITION),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_SCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetArmCommand(armSubsystem, ARM_MIDDLE_POSITION),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_SCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetArmCommand(armSubsystem, ARM_SUBSTATION_POSITION),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_SCORE),
				new WaitCommand(0.5),
				new SetArmCommand(armSubsystem, ARM_HIGH_POSITION),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CUBE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CONE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_SCORE),
				new WaitCommand(0.5),
				new SetWristCommand(armSubsystem, intakeSubsystem, WRIST_RETRACT_CUBE));
	}
}
