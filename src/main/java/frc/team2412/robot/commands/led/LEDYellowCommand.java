package frc.team2412.robot.commands.led;

import static frc.team2412.robot.subsystems.LEDSubsystem.LEDConstants.*;

import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDYellowCommand extends LEDSetCommand {

	public LEDYellowCommand(LEDSubsystem ledSubsystem) {
		super(ledSubsystem, yR, yG, yB);
	}
}
