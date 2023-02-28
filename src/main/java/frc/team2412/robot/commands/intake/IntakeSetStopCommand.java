package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetStopCommand extends IntakeSetCommand {

	public IntakeSetStopCommand(IntakeSubsystem intakeSubsystem) {
		super(intakeSubsystem, 0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
