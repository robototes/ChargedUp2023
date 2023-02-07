package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class IntakeConstants {
		// speeds
		public static final double INTAKE_IN_SPEED = 0.5;
		public static final double INTAKE_OUT_SPEED = 0.3;

		public static final double INTAKE_CUBE_DISTANCE = 0;
		public static final double INTAKE_CONE_DISTANCE = 0;
		public static final double INTAKE_CUBE_COLOR = 0;
		public static final double INTAKE_CONE_COLOR = 141;

		// enums
		public static enum GamePieceType {
			CUBE,
			CONE,
			NONE;

			/*
			 * for reference hi cammy eggy
			 * String example;
			 *
			 * GamePieceType(String example) {
			 * this.example = example;
			 * }
			 */

		}

		// public final cone;
		// public final cube;
		// public final nothing;

	}
	// HARDWARE

	private final CANSparkMax motor;
	// private final ColorSensorV3 colorSensor;
	private final AnalogInput distanceSensor;

	// CONSTRUCTOR
	public IntakeSubsystem() {
		motor = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
		// colorSensor = new ColorSensorV3(Port.kOnboard); //to find I2C port
		// distanceSensor = new ();
		motor.setIdleMode(IdleMode.kBrake);
		distanceSensor = new AnalogInput(INTAKE_DISTANCE_SENSOR);
	}

	// METHODS
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	public double getSpeed() {
		return motor.get();
	}

	public void intakeIn() {
		setSpeed(INTAKE_IN_SPEED);
	}

	public void intakeOut() {
		setSpeed(INTAKE_OUT_SPEED);
	}

	public void intakeStop() {
		setSpeed(0);
	}

	public GamePieceType detectType() {
		// if () {
		//     return GamePieceType.CUBE;
		// }
		// else if () {
		//     return GamePieceType.CONE;
		// }

		return GamePieceType.NONE;
	}

	public boolean isSecured() { // Checks to see if the game piece is secured
		return false;
	}

	public void rumble() {}

	public double getDistance() {
		return Math.pow(distanceSensor.getAverageVoltage(), -1.2045) * 27.726;
	}
}
