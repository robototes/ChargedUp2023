package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private static final double CUBE_LED_COLOR = 0.69;
	private static final double CONE_LED_COLOR = 0.91;
	private static final double RED_LED_COLOR = 0.61;

	private final PWM blinkin;

	public LEDSubsystem() {
		blinkin = new PWM(BLINKIN_LED);
		blinkin.setSpeed(RED_LED_COLOR); // sets to red
	}

	public void setLED(double color) {
		blinkin.setSpeed(color);
		System.out.println("set color " + color);
	}

	public void setLEDCube() {
		blinkin.setSpeed(CUBE_LED_COLOR);
	}

	public void setLEDCone() {
		blinkin.setSpeed(CONE_LED_COLOR);
	}
}
