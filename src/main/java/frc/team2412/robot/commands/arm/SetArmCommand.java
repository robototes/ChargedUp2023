package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;

public class SetArmCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private PositionType positionType;
	private double tolerance;

	public SetArmCommand(ArmSubsystem armSubsystem, PositionType positionType, double tolerance) {
		this.armSubsystem = armSubsystem;
		this.positionType = positionType;
		this.tolerance = tolerance;
	}

	@Override
	public void initialize() {
		if (!armSubsystem.getManualOverride()) {
			armSubsystem.setPosition(positionType);
			armSubsystem.setArmGoal(positionType.armAngle);
		}
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopArm();
	}

	@Override
	public boolean isFinished() {
		return armSubsystem.isArmNearGoal(tolerance);
	}
}
