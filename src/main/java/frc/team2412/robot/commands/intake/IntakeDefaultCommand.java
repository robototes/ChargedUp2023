package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.INTAKE_HOLD_SPEED;

import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType;

public class IntakeDefaultCommand extends IntakeSetCommand {

	public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem) {

		super(
				intakeSubsystem,
				intakeSubsystem.detectType() == GamePieceType.NONE ? 0 : INTAKE_HOLD_SPEED);
	}
}
