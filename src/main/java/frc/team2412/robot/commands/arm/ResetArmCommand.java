package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class ResetArmCommand extends CommandBase {
	private final ArmSubsystem arm;

	private final double ARM_RETRACT_SPEED = -0.1;

	public ResetArmCommand(ArmSubsystem arm) {
		this.arm = arm;
	}

	@Override
	public void initialize() {
		arm.setManualOverride(true);
		arm.disableShoulderLimits();
	}

	@Override
	public void execute() {
		arm.setArmMotor(ARM_RETRACT_SPEED);
	}

	@Override
	public void end(boolean interrupted) {
		arm.resetArmEncoder();
		arm.setManualOverride(false);
		arm.enableShoulderLimits();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
