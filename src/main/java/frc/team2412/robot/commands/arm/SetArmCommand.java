package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetArmCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private double targetAngle;

	public SetArmCommand(ArmSubsystem armSubsystem, double targetAngle) {
		this.armSubsystem = armSubsystem;
		this.targetAngle = targetAngle;
		addRequirements(armSubsystem);
	}

	@Override
	public void execute() {
		armSubsystem.rotateArmTo(targetAngle);
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.stopArm();
	}

	@Override
	public boolean isFinished() {
		return armSubsystem.armIsAtGoal();
	}
}
