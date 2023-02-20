package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualArmOverrideCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private DoubleSupplier armJoystickInput;
	private DoubleSupplier wristJoystickInput;

	public ManualArmOverrideCommand(
			ArmSubsystem armSubsystem,
			DoubleSupplier armJoystickInput,
			DoubleSupplier wristJoystickInput) {
		this.armSubsystem = armSubsystem;
		this.armJoystickInput = armJoystickInput;
		this.wristJoystickInput = wristJoystickInput;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		armSubsystem.setManualOverride(true);
		System.out.println("init manual control");
	}

	@Override
	public void execute() {
		armSubsystem.setArmMotor(deadbandCorrection(armJoystickInput.getAsDouble()));
		armSubsystem.setWristMotor(deadbandCorrection(wristJoystickInput.getAsDouble()));
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.setManualOverride(false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public double deadbandCorrection(double input) {
		return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
	}
}
