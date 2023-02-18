package frc.team2412.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
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
		armSubsystem.setArmMotor(MathUtil.applyDeadband(armJoystickInput.getAsDouble(), 0.05));
		armSubsystem.setWristMotor(MathUtil.applyDeadband(wristJoystickInput.getAsDouble(), 0.05));
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.setManualOverride(false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
