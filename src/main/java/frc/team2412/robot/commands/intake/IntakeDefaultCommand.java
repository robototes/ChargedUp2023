package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends IntakeSetCommand {

	public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem) {

		super(intakeSubsystem, 0.4);
	}

	@Override
	public void execute() {
		// intakeSubsystem.setSpeed(intakeSubsystem.getHoldSpeed());
		// set motor speed using super initialize?
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
