package frc.team2412.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.LEDSubsystem;

public class LEDSetCommand extends CommandBase {
	private LEDSubsystem ledSubsystem;
	// private AddressableLEDBuffer buffer; might use in future
	private int r;
	private int g;
	private int b;

	public LEDSetCommand(LEDSubsystem ledSubsystem, int r, int g, int b) {
		this.ledSubsystem = ledSubsystem;
		this.r = r;
		this.g = g;
		this.b = b;
		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		// set buffer to buffer fr
		ledSubsystem.setBuffer(r, g, b);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
