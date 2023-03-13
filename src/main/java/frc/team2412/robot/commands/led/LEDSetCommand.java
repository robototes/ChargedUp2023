package frc.team2412.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDSetCommand extends CommandBase {
	private LEDSubsystem ledSubsystem;
	// private AddressableLEDBuffer buffer; might use in future
	private double color;

	public LEDSetCommand(LEDSubsystem ledSubsystem, double color) {
		this.ledSubsystem = ledSubsystem;
		this.color = color;
		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		// set buffer to buffer fr
		ledSubsystem.setLED(color);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
