package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class ManualArmOverrideCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private double armJoystickInput;
	private double wristJoystickInput;

	public ManualArmOverrideCommand(
			ArmSubsystem armSubsystem, double armJoystickInput, double wristJoystickInput) {
		this.armSubsystem = armSubsystem;
		this.armJoystickInput = armJoystickInput;
		this.wristJoystickInput = wristJoystickInput;
	}

	@Override
	public void initialize() {
		armSubsystem.setManualOverride(true);
	}

	@Override
	public void execute() {
		armSubsystem.setArmMotor(armJoystickInput);
		armSubsystem.setWristMotor(wristJoystickInput);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
