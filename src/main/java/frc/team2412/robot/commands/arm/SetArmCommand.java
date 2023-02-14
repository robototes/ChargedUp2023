package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;

public class SetArmCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private PositionType positionType;

	public SetArmCommand(ArmSubsystem armSubsystem, PositionType positionType) {
		this.armSubsystem = armSubsystem;
		this.positionType = positionType;
	}

	@Override
	public void initialize() {
		armSubsystem.setPosition(positionType);
		armSubsystem.setArmGoal(positionType.armAngle);
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopArm();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
