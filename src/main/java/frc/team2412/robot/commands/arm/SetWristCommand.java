package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType;

public class SetWristCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private WristPosition targetWristPosition;

	public static enum WristPosition {
		WRIST_RETRACT,
		WRIST_RETRACT_CUBE,
		WRIST_RETRACT_CONE,
		WRIST_PRESCORE,
		WRIST_SCORE;
	}

	public SetWristCommand(
			ArmSubsystem armSubsystem,
			IntakeSubsystem intakeSubsystem,
			WristPosition targetWristPosition) {
		this.armSubsystem = armSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.targetWristPosition = targetWristPosition;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		double targetWristAngle = 0.0;

		if (armSubsystem.getPosition() == PositionType.UNKNOWN_POSITION) {
			// return;
		}

		System.out.println(armSubsystem.getPosition());

		switch (targetWristPosition) {
			case WRIST_RETRACT:
				if (intakeSubsystem.detectType() == GamePieceType.CONE) {
					targetWristAngle = armSubsystem.getPosition().retractedConeWristAngle;
				} else {
					targetWristAngle = armSubsystem.getPosition().retractedWristAngle;
				}
				break;
			case WRIST_RETRACT_CUBE:
				targetWristAngle = armSubsystem.getPosition().retractedWristAngle;
				break;
			case WRIST_RETRACT_CONE:
				targetWristAngle = armSubsystem.getPosition().retractedConeWristAngle;
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
