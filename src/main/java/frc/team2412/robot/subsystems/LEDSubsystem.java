package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;
import static frc.team2412.robot.subsystems.LEDSubsystem.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;

public class LEDSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class LEDConstants {
		public static final int BUFFER_LENGTH = 72;
	}

	private final AddressableLED leds;
	private final AddressableLEDBuffer buffer;

	public LEDSubsystem() {
		// TODO: maybe pass in buffer length
		leds = new AddressableLED(LED_STRIP); // initialization of the AdressableLED
		leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once

		// creates buffer
		buffer = new AddressableLEDBuffer(BUFFER_LENGTH);

		leds.start();

		// TODO we start a starting color
	}

	public int getBufferLength() {
		return BUFFER_LENGTH;
	} // End of getBufferLength()

	public AddressableLEDBuffer getBuffer() {
		return buffer;
	}

	public void setBuffer(Color color) {
		for (var i = 0; i < buffer.getLength(); i++) {
			// Sets each led in the buffer to the color
			buffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
		}
		// Sets the led to the buffer
		leds.setData(buffer);
	}
}
