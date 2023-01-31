package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetWristCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private double targetAngle;

	public SetWristCommand(ArmSubsystem armSubsystem, double targetAngle) {
		this.armSubsystem = armSubsystem;
		this.targetAngle = targetAngle;
		addRequirements(armSubsystem);
	}

	@Override
	public void execute() {
		armSubsystem.rotateWristTo(targetAngle);
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.stopWrist();
	}

	@Override
	public boolean isFinished() {
		return armSubsystem.wristIsAtGoal();
	}
}
