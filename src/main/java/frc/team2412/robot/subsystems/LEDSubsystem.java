package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;
import static frc.team2412.robot.subsystems.LEDSubsystem.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class LEDConstants {
		public static final int BUFFER_LENGTH = 72;

		// private final RGB PURPLE_RGB = 145, 48, 255;
		// public static final int pR = 145;
		// public static final int pG = 48;
		// public static final int pB = 255;
		// private final RGB YELLOW_RGB = 255, 245, 45;
		// public static final int yR = 255;
		// public static final int yG = 245;
		// public static final int yB = 45;
	}

	private final AddressableLED leds;
	private final AddressableLEDBuffer buffer;

	public LEDSubsystem() {
		// TODO maybe pass in buffer length
		leds = new AddressableLED(LED_STRIP); // initialization of the AdressableLED
		leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once

		// creates buffer
		buffer = new AddressableLEDBuffer(BUFFER_LENGTH);

		// TODO we start a starting color
	}

	public int getBufferLength() {
		return BUFFER_LENGTH;
	} // End of getBufferLength()

	public AddressableLEDBuffer getBuffer() {
		return buffer;
	}

	public void setBuffer(int r, int g, int b) {
		for (var i = 0; i < buffer.getLength(); i++) {
			// Sets each led in the buffer to the color
			buffer.setRGB(i, r, g, b);
		}
		// Sets the led to the buffer
		leds.setData(buffer);
	}
}
