package frc.team2412.robot.commands.led;

import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;
import static frc.team2412.robot.subsystems.LEDSubsystem.LEDConstants.*;

import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDPurpleCommand extends LEDSetCommand {

	public LEDPurpleCommand(LEDSubsystem ledSubsystem) {
		super(ledSubsystem, CUBE.ledColor);
	}
}
