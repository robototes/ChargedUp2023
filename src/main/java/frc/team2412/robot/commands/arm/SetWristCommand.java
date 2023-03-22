package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetWristCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private WristPosition targetWristPosition;

	public static enum WristPosition {
		WRIST_RETRACT,
		WRIST_PRESCORE,
		WRIST_SCORE;
	}

	public SetWristCommand(
			ArmSubsystem armSubsystem,
			WristPosition targetWristPosition) {
		this.armSubsystem = armSubsystem;
		this.targetWristPosition = targetWristPosition;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		double targetWristAngle = 0.0;

		switch (targetWristPosition) {
			case WRIST_RETRACT:
				targetWristAngle = armSubsystem.getPosition().retractedWristAngle;
				break;
			case WRIST_PRESCORE:
				targetWristAngle = armSubsystem.getPosition().prescoringWristAngle;
				break;
			case WRIST_SCORE:
				targetWristAngle = armSubsystem.getPosition().scoringWristAngle;
				break;
		}

		armSubsystem.setWristGoal(targetWristAngle);
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopWrist();
	}

	@Override
	public boolean isFinished() {
		// has pid moved us close enough to go back to manual control?
		// return Math.abs(armSubsystem.getWristPosition() - targetWristAngle) < 0.1;
		return true;
	}
}
