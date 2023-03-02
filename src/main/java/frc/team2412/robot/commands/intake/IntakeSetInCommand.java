package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED;

import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetInCommand extends IntakeSetCommand {

	public IntakeSetInCommand(IntakeSubsystem intakeSubsystem) {
		super(intakeSubsystem, INTAKE_IN_SPEED);
	}

	// TODO: uncomment when color matching works
	@Override
	public boolean isFinished() {
		return intakeSubsystem.isSecured();
	}
}
