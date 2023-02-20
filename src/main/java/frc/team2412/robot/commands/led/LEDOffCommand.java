package frc.team2412.robot.commands.led;

import static frc.team2412.robot.subsystems.LEDSubsystem.LEDConstants.*;

import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDOffCommand extends LEDSetCommand {

	public LEDOffCommand(LEDSubsystem ledSubsystem) {
		super(ledSubsystem, 0, 0, 0);
	}
}
