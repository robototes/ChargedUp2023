package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class SetWristCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private double targetAngle;
	private WristPosition wristPosition;

	public static enum WristPosition {
		RETRACT,
		RETRACT_CUBE,
		RETRACT_CONE,
		PRESCORE,
		SCORE;
	}

	public SetWristCommand(ArmSubsystem armSubsystem, WristPosition wristPosition) {
		this.armSubsystem = armSubsystem;
		this.wristPosition = wristPosition;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {

		switch (wristPosition) {
			case RETRACT:
				targetAngle =
						0; // TODO: get intake gamePiece and decide which retract value to use based off said
				// piece
				break;
			case RETRACT_CUBE:
				targetAngle = armSubsystem.getPosition().retractedWristAngle;
				break;
			case RETRACT_CONE:
				targetAngle = armSubsystem.getPosition().retractedConeWristAngle;
				break;

			case PRESCORE:
				targetAngle = armSubsystem.getPosition().prescoringWristAngle;
				break;

			case SCORE:
				targetAngle = armSubsystem.getPosition().scoringWristAngle;
				break;
		}

		armSubsystem.setWristGoal(targetAngle);
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopWrist();
	}

	@Override
	public boolean isFinished() {
		return armSubsystem.wristIsAtGoal();
	}
}
