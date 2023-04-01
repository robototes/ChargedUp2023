package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetFastOutCommand extends CommandBase {
	IntakeSubsystem intakeSubsystem;

	public IntakeSetFastOutCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeFastOut();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
