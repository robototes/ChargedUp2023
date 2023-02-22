package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualArmOverrideOffCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private DoubleSupplier armJoystickInput;
	private DoubleSupplier wristJoystickInput;

	public ManualArmOverrideOffCommand(
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
		armSubsystem.setManualOverride(false);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
