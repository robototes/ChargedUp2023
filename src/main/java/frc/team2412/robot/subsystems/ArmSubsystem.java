package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;

public class ArmSubsystem extends SubsystemBase {

	// Constants

	public static class ArmConstants { // To find values later
		// Mech problem basically

		// pounds/lbs
		public static final double INNER_ARM_MASS = 4.69;
		public static final double OUTER_ARM_MASS = 5.03;
		public static final double INTAKE_MASS = 5.57;

		// Center of mass stuff

		// inches
		public static final double INNER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT = 11.218;
		public static final double OUTER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT = 15.712;
		public static final double WRIST_CENTER_OF_MASS_DISTANCE_FROM_JOINT =
				10.85; // TODO: find distance
		public static final double WRIST_CENTER_OF_MASS_ANGLE_OFFSET = -20.15; // TODO: find offset

		public static final double INNER_ARM_LENGTH = 25;
		public static final double OUTER_ARM_LENGTH = 27.25;

		public static final double SHOULDER_ELBOW_GEAR_RATIO = 68 / 48;

		// PID
		// TODO: FIND ALL VALUES
		public static final double ARM_K_P = 0;
		public static final double ARM_K_I = 0;
		public static final double ARM_K_D = 0;

		public static final double WRIST_K_P = 0;
		public static final double WRIST_K_I = 0;
		public static final double WRIST_K_D = 0;

		// Feed Forward
		public static final double ARM_K_A = 0;
		public static final double ARM_K_G = 0;
		public static final double ARM_K_S = 0;
		public static final double ARM_K_V = 0;

		public static final double WRIST_K_A = 0;
		public static final double WRIST_K_G = 0;
		public static final double WRIST_K_S = 0;
		public static final double WRIST_K_V = 0;

		// Constraints
		public static final int MIN_PERCENT_OUTPUT = -1;
		public static final int MAX_PERCENT_OUTPUT = 1;

		public static final double ARM_POS_TOLERANCE = 0.01;
		public static final double WRIST_POS_TOLERANCE = 0.01;

		public static final double ARM_VELOCITY_TOLERANCE = 0.2;
		public static final double WRIST_VELOCITY_TOLERANCE = 0.2;

		public static final double MAX_ARM_VELOCITY = 5;
		public static final double MAX_ARM_ACCELERATION = 5;

		public static final double MAX_WRIST_VELOCITY = 5;
		public static final double MAX_WRIST_ACCELERATION = 5;

		public static final Constraints ARM_CONSTRAINTS =
				new Constraints(MAX_ARM_ACCELERATION, MAX_ARM_VELOCITY);
		public static final Constraints WRIST_CONSTRAINTS =
				new Constraints(MAX_WRIST_ACCELERATION, MAX_WRIST_VELOCITY);

		public static final double TICKS_TO_INCHES = 42 / 360; // Maybe unneeded?

		// TODO: find limits
		public static final double MIN_ARM_ANGLE = 56.9;
		public static final double MAX_ARM_ANGLE = 170;
		public static final double MIN_WRIST_ANGLE = 20;
		public static final double MAX_WRIST_ANGLE = 100;

		// ENUM

		/*
		 * TODO:
		 * Find arm and wrist angles (see onenote page)
		 *
		 * Values are angles that correspond to specific arm positions:
		 * Arm Angle
		 * Retracted Wrist Angle
		 * Retracted Wrist Angle (cone edition)
		 * Prescore Wrist Angle
		 * Scoring Wrist Angle
		 */
		public static enum PositionType {
			ARM_LOW_POSITION(0, 0, 279.6, 0, 0),
			ARM_MIDDLE_POSITION(87.42, 0, 279.6, 0, 80.27),
			ARM_HIGH_POSITION(50, 0, 279.6, 0, 111),
			ARM_SUBSTATION_POSITION(80, 0, 279.6, 0, 0); // ?

			public final double armAngle;
			public final double retractedWristAngle;
			public final double retractedConeWristAngle;
			public final double prescoringWristAngle;
			public final double scoringWristAngle;

			PositionType(
					double armAngle,
					double retractedWristAngle,
					double retractedConeWristAngle,
					double prescoringWristAngle,
					double scoringWristAngle) {
				this.armAngle = armAngle;
				this.retractedWristAngle = retractedWristAngle;
				this.retractedConeWristAngle = retractedConeWristAngle;
				this.prescoringWristAngle = prescoringWristAngle;
				this.scoringWristAngle = scoringWristAngle;
			}
		}
	}

	// Hardware

	private PositionType currentPosition;
	private boolean manualOverride;

	private final CANSparkMax armMotor1;
	private final CANSparkMax armMotor2;
	private final CANSparkMax wristMotor;

	private final Encoder shoulderEncoder;
	private final Encoder wristEncoder;

	private final ProfiledPIDController armPID;
	private final ProfiledPIDController wristPID;

	private final ArmFeedforward wristFeedforward;

	// Constructor

	public ArmSubsystem() {
		armMotor1 = new CANSparkMax(ARM_MOTOR1, MotorType.kBrushless);
		armMotor2 = new CANSparkMax(ARM_MOTOR2, MotorType.kBrushless);
		wristMotor = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);

		shoulderEncoder = new Encoder(SHOULDER_ENCODER_PORT_A, SHOULDER_ENCODER_PORT_B);
		wristEncoder = new Encoder(WRIST_ENCODER_PORT_A, WRIST_ENCODER_PORT_B);

		armPID = new ProfiledPIDController(ARM_K_P, ARM_K_I, ARM_K_D, ARM_CONSTRAINTS);
		wristPID = new ProfiledPIDController(WRIST_K_P, WRIST_K_I, WRIST_K_D, WRIST_CONSTRAINTS);

		wristFeedforward = new ArmFeedforward(WRIST_K_A, WRIST_K_G, WRIST_K_S, WRIST_K_V);

		armMotor2.follow(armMotor1, true);

		armMotor1.setIdleMode(IdleMode.kBrake);
		armMotor2.setIdleMode(IdleMode.kBrake);
		wristMotor.setIdleMode(IdleMode.kBrake);

		armPID.setTolerance(ARM_POS_TOLERANCE, ARM_VELOCITY_TOLERANCE);
		wristPID.setTolerance(WRIST_POS_TOLERANCE, WRIST_VELOCITY_TOLERANCE);

		currentPosition = ARM_LOW_POSITION;
		manualOverride = false;

		shoulderEncoder.reset();
		wristEncoder.reset();

		armPID.reset(getShoulderAngle());
		wristPID.reset(getWristAngle());
	}

	// Methods

	public void setArmMotor(double percentOutput) {
		armMotor1.set(
				MathUtil.clamp(MAX_ARM_VELOCITY * percentOutput, MIN_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT));
	}

	public void setWristMotor(double percentOutput) {
		wristMotor.set(
				MathUtil.clamp(MAX_WRIST_VELOCITY + percentOutput, MIN_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT));
	}

	public void setPosition(PositionType position) {
		currentPosition = position;
	}

	public void setManualOverride(boolean override) {
		manualOverride = override;
	}

	public void setArmGoal(double targetAngle) {
		armPID.setGoal(MathUtil.clamp(targetAngle, MIN_ARM_ANGLE, MAX_ARM_ANGLE));
	}

	public void setWristGoal(double targetAngle) {
		wristPID.setGoal(MathUtil.clamp(targetAngle, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE));
	}

	public double calculateArmPID() {
		return armPID.calculate(shoulderEncoder.getDistance(), armPID.getGoal());
	}

	public double calculateWristPID() {
		return armPID.calculate(wristEncoder.getDistance(), wristPID.getGoal());
	}

	public double calculateArmFeedforward() {
		return ARM_K_S * Math.signum(armPID.getGoal().position - getShoulderAngle())
				+ ARM_K_G * getAngleTowardsCenterOfMass()
				+ ARM_K_V * 0
				+ ARM_K_A * 0;
	}

	public double calculateWristFeedforward() {
		return wristFeedforward.calculate(
				shoulderEncoder.getDistance() + wristEncoder.getDistance(), 0); // getRate = getVelocity?
	}

	public double getShoulderAngle() {
		return shoulderEncoder.getDistance();
	}

	public double getElbowAngle() {
		return shoulderEncoder.getDistance() * SHOULDER_ELBOW_GEAR_RATIO;
	}

	public double getWristAngle() {
		return wristEncoder.getDistance();
	}

	public PositionType getPosition() {
		return currentPosition;
	}

	public double convertToVolts(double percentOutput) {
		return percentOutput * Robot.getInstance().getVoltage();
	}

	public double getAngleTowardsCenterOfMass() {

		// Get Coordinates of Centers of Mass
		double innerArmCenterOfMassX = 11.218 * Math.cos(Math.toRadians(180 - getShoulderAngle()));
		double innerArmCenterOfMassY = 11.218 * Math.sin(Math.toRadians(180 - getShoulderAngle()));

		double outerArmCenterOfMassX =
				15.722 * Math.cos(Math.toRadians(getElbowAngle() - getShoulderAngle()))
						+ INNER_ARM_LENGTH * Math.cos(Math.toRadians(180 - getShoulderAngle()));
		double outerArmCenterOfMassY =
				15.722 * Math.sin(Math.toRadians(getElbowAngle() - getShoulderAngle()))
						+ INNER_ARM_LENGTH * Math.sin(Math.toRadians(180 - getShoulderAngle()));

		double intakeCenterOfMassX =
				0 * Math.cos(Math.toRadians(180 - (getWristAngle() + WRIST_CENTER_OF_MASS_ANGLE_OFFSET)))
						+ OUTER_ARM_LENGTH * Math.cos(Math.toRadians(getElbowAngle() - getShoulderAngle()))
						+ INNER_ARM_LENGTH * Math.cos(Math.toRadians(180 - getShoulderAngle()));
		double intakeCenterOfMassY =
				0 * Math.sin(Math.toRadians(180 - (getWristAngle() + WRIST_CENTER_OF_MASS_ANGLE_OFFSET)))
						+ OUTER_ARM_LENGTH * Math.sin(Math.toRadians(getElbowAngle() - getShoulderAngle()))
						+ INNER_ARM_LENGTH * Math.sin(Math.toRadians(180 - getShoulderAngle()));

		// Calculate Average Center of Mass of Arm + Intake
		double centerOfMassX =
				(INNER_ARM_MASS * innerArmCenterOfMassX
								+ OUTER_ARM_MASS * outerArmCenterOfMassX
								+ INTAKE_MASS * intakeCenterOfMassX)
						/ (INNER_ARM_MASS + OUTER_ARM_MASS + INTAKE_MASS);
		double centerOfMassY =
				(INNER_ARM_MASS * innerArmCenterOfMassY
								+ OUTER_ARM_MASS * outerArmCenterOfMassY
								+ INTAKE_MASS * intakeCenterOfMassY)
						/ (INNER_ARM_MASS + OUTER_ARM_MASS + INTAKE_MASS);

		// Return Angle needed to face Center of Mass
		return Math.atan2(centerOfMassY, centerOfMassX);
	}

	@Override
	public void periodic() {
		// Periodic Arm movement for Preset Angle Control
		if (!manualOverride) {
			armMotor1.setVoltage(
					convertToVolts(
							MathUtil.clamp(
									calculateArmPID() + calculateArmFeedforward(),
									MIN_PERCENT_OUTPUT,
									MAX_PERCENT_OUTPUT)));
			wristMotor.setVoltage(
					convertToVolts(
							MathUtil.clamp(
									calculateWristPID() + calculateWristFeedforward(),
									MIN_PERCENT_OUTPUT,
									MAX_PERCENT_OUTPUT)));
		}
	}
}
