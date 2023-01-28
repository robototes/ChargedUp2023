package frc.team2412.robot.commands.arm;

import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.RETRACT_ARM_ANGLE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.RETRACT_WRIST_ANGLE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class RetractArmCommand extends CommandBase {

	private ArmSubsystem armSubsystem;

	public RetractArmCommand(ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		addRequirements(armSubsystem);
	}

	@Override
	public void execute() {
		armSubsystem.rotateArmTo(RETRACT_ARM_ANGLE);
		armSubsystem.rotateWristTo(RETRACT_WRIST_ANGLE);
	}

	@Override
	public void end(boolean interrupted) {
		// might not need, might be bad.
		armSubsystem.stopArm();
		armSubsystem.stopWrist();
	}

	@Override
	public boolean isFinished() {
		return (armSubsystem.armIsAtGoal() && armSubsystem.wristIsAtGoal());
	}
}
