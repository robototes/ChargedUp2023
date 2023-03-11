package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetCommand extends CommandBase {

	protected IntakeSubsystem intakeSubsystem;
	private double speed;

	public IntakeSetCommand(IntakeSubsystem intakeSubsystem, double speed) {
		this.intakeSubsystem = intakeSubsystem;
		this.speed = speed;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		// set motor speed to speed
		intakeSubsystem.setSpeed(speed);
	}

	@Override
	public void execute() {
		// set motor speed to speed
		intakeSubsystem.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
