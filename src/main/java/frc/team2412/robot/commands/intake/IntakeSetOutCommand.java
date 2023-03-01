package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.INTAKE_OUT_SPEED;

import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetOutCommand extends IntakeSetCommand {

	public IntakeSetOutCommand(IntakeSubsystem intakeSubsystem) {
		super(intakeSubsystem, INTAKE_OUT_SPEED);
	}

	@Override
	public boolean isFinished() {
		// TODO: add or statement for if distance sensor detects piece is out
		// return (intakeSubsystem.detectType() == GamePieceType.NONE);
		return false;
	}
}
