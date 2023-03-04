package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class ResetIntakeMotorsCommand extends CommandBase {

	private IntakeSubsystem intakeSubsystem;

	public ResetIntakeMotorsCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
	}

	public void initialize() {
		intakeSubsystem.resetMotors();
	}
}
