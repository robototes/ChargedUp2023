package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;

public class IntakeSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class IntakeConstants {
		// speeds
<<<<<<< HEAD
		public static final double INTAKE_IN_SPEED = 0.3;
		public static final double INTAKE_OUT_SPEED = -0.1;
=======
		public static final double INTAKE_HOLD_SPEED = 0.1;
		public static final double INTAKE_IN_SPEED = 0.8;
		public static final double INTAKE_OUT_SPEED = -0.8;
>>>>>>> main

		public static final double INTAKE_CUBE_DISTANCE = 0;
		public static final double INTAKE_CONE_DISTANCE = 0;

		public static final int INTAKE_COLOR_THRESHOLD = 10;

		// enums

		public static enum GamePieceType {
			CUBE(new Color(145, 48, 255), 15),
			CONE(new Color(255, 245, 45), 12),
			NONE(new Color(0, 0, 0), 0);

			public final Color color;
			// TODO: find distance from sensor values
			public final double distanceFromSensor;

			GamePieceType(Color color, double distanceFromSensor) {
				this.color = color;
				this.distanceFromSensor = distanceFromSensor;
			}
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

		colorSensor = new ColorSensorV3(Port.kOnboard);
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
	}

	public void intakeStop() {
		setSpeed(0);
	}

	public GamePieceType detectType() {
		if (colorSensorEquals(CUBE.color)) {
			return GamePieceType.CUBE;
		} else if (colorSensorEquals(CONE.color)) {
			return GamePieceType.CONE;
		}
		return GamePieceType.NONE;
	}

	public boolean colorSensorEquals(Color color) {
		// r
		if (colorSensor.getRed() <= (color.getRed() + INTAKE_COLOR_THRESHOLD)
				&& colorSensor.getRed() >= (color.getRed() - INTAKE_COLOR_THRESHOLD)) {
			// g
			if (colorSensor.getGreen() <= (color.getGreen() + INTAKE_COLOR_THRESHOLD)
					&& colorSensor.getGreen() >= (color.getGreen() - INTAKE_COLOR_THRESHOLD)) {
				// b
				if (colorSensor.getBlue() <= (color.getBlue() + INTAKE_COLOR_THRESHOLD)
						&& colorSensor.getBlue() >= (color.getBlue() - INTAKE_COLOR_THRESHOLD)) {
					return true;
				}
			}
		}
		return false;
	}

	public boolean isSecured() {
		// Checks to see if the game piece is secured, returns true if the motor should stop
		return (getDistance() < GamePieceType.CONE.distanceFromSensor
				|| (detectType() == GamePieceType.CUBE
						&& getDistance() < GamePieceType.CUBE.distanceFromSensor));
	}

	public double getDistance() {
		// equation found from docs to convert voltage to cm
		return Math.pow(distanceSensor.getAverageVoltage(), -1.2045) * 27.726;
	}
}
