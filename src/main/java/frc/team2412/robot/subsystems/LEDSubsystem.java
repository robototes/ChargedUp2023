package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private final int BUFFER_LENGTH = 72;
	private final AddressableLED leds;
	private final AddressableLEDBuffer buffer;

	public LEDSubsystem(int port) {
		// TODO maybe pass in buffer length
		leds = new AddressableLED(port); // initialization of the AdressableLED
		leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once
		buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
		// TODO we start a starting color
		setBuffer(buffer);
	}

	public int getBufferLength() {
		return BUFFER_LENGTH;
	} // End of getBufferLength()

	public AddressableLEDBuffer getBuffer() {
		return buffer;
	}

	public void setBuffer(AddressableLEDBuffer buffer) {
		leds.setData(buffer);
	}
}
