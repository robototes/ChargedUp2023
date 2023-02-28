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

	public void setSpeed(double percentOutput) {
		speed = percentOutput;
	}

	@Override
	public void initialize() {
		System.out.println(speed);
		// set motor speed to speed
		intakeSubsystem.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
