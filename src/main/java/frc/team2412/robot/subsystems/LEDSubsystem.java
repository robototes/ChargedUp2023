package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private final PWM blinkin;

	public LEDSubsystem() {
		blinkin = new PWM(BLINKIN_LED);
	}

	public void setLED(double color) {
		blinkin.setSpeed(color);
	}
}
