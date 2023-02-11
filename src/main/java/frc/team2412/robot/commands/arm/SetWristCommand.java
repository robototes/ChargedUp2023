package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetWristCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private double targetWristAngle;

	public static enum WristPosition {
		WRIST_RETRACT,
		WRIST_RETRACT_CUBE,
		WRIST_RETRACT_CONE,
		WRIST_PRESCORE,
		WRIST_SCORE;
	}

	public SetWristCommand(ArmSubsystem armSubsystem, WristPosition targetWristPosition) {
		this.armSubsystem = armSubsystem;

		switch (targetWristPosition) {
			case WRIST_RETRACT:
				targetWristAngle =
						0; // TODO: get intake gamePiece and decide which retract value to use based off said
				// piece
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

		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		armSubsystem.setWristGoal(targetWristAngle);
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopWrist();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
