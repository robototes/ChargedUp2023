package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	public static final double LED_PURPLE = 0.91;
	public static final double LED_YELLOW = 0.69;
	public static final double LED_OFF = 0.99;
	public static final double LED_RED = 0.61;
	
	private final PWM blinkin;

	public LEDSubsystem() {
		blinkin = new PWM(BLINKIN_LED);
		blinkin.setSpeed(LED_RED); // sets to red
	}

	public void setLED(double color) {
		blinkin.setSpeed(color);
		System.out.println("set color " + color);
	}
}
