package frc.team2412.robot.commands.led;

import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;

import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDOffCommand extends LEDSetCommand {

	public LEDOffCommand(LEDSubsystem ledSubsystem) {
		super(ledSubsystem, NONE.ledColor);
	}
}
