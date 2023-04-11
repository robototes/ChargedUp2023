package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetWristAngleCommand extends CommandBase {
	private final ArmSubsystem armSubsystem;
	private final double wristAngle;

	public SetWristAngleCommand(ArmSubsystem armSubsystem, double wristAngle) {
		this.armSubsystem = armSubsystem;
		this.wristAngle = wristAngle;
	}

	@Override
	public void initialize() {
		armSubsystem.setWristGoal(wristAngle);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(armSubsystem.getWristPosition() - wristAngle) < 0.05;
	}
}
