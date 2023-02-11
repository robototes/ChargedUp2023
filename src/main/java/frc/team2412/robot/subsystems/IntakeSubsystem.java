package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class IntakeConstants {
		// speeds
		public static final double INTAKE_IN_SPEED = 0.8;
		public static final double INTAKE_OUT_SPEED = -0.8;

		public static final double INTAKE_CUBE_DISTANCE = 0;
		public static final double INTAKE_CONE_DISTANCE = 0;
		public static final Color INTAKE_CUBE_COLOR = new Color(145, 48, 255);
		public static final Color INTAKE_CONE_COLOR = new Color(255, 245, 45);
		public static final Color INTAKE_OUT_COLOR = new Color(0, 0, 0);

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

		// public final distance;

	}
	// HARDWARE

	private final CANSparkMax motor1;
	private final CANSparkMax motor2;
	private final ColorSensorV3 colorSensor;
	private final AnalogInput distanceSensor;

	// CONSTRUCTOR
	public IntakeSubsystem() {
		motor1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
		motor2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushless);
		motor1.setIdleMode(IdleMode.kBrake);
		motor2.setIdleMode(IdleMode.kBrake);

		colorSensor = new ColorSensorV3(Port.kMXP);
		distanceSensor = new AnalogInput(INTAKE_DISTANCE_SENSOR);
	}

	// METHODS
	public void setSpeed(double speed) {
		motor1.set(speed);
		motor2.set(-speed);
	}

	public double getSpeed() {
		return motor1.get();
	}

	public void intakeIn() {
		setSpeed(INTAKE_IN_SPEED);
	}

	public void intakeOut() {
		setSpeed(INTAKE_OUT_SPEED);
		setColor(INTAKE_OUT_COLOR);
	}

	public void intakeStop() {
		setSpeed(0);
	}
	

	public GamePieceType detectType() {
		if (colorSensor.getColor().equals(INTAKE_CUBE_COLOR)) {
			return GamePieceType.CUBE;
		} else if (colorSensor.getColor().equals(INTAKE_CONE_COLOR)) {
			return GamePieceType.CONE;
		}
		return GamePieceType.NONE;
	}

	public boolean
			isSecured() { // Checks to see if the game piece is secured, returns true if the motor should
		// stop
		if (getDistance() < 12) { // units in cm
			return true;
		} else if (detectType() == GamePieceType.CUBE
				&& getDistance() < 15) { // also arbitrary numbers right now
			return true;
		}
		return false;
	}

	public double getDistance() {
		return Math.pow(distanceSensor.getAverageVoltage(), -1.2045)
				* 27.726; // equation found from docs to convert voltage to cm
	}
 
}
