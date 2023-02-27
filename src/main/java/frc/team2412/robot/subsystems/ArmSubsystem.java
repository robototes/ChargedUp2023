package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
		public static final double WRIST_CENTER_OF_MASS_DISTANCE_FROM_JOINT = 10.85;
		public static final double WRIST_CENTER_OF_MASS_ANGLE_OFFSET = -20.15; // TODO: find offset

		public static final double INNER_ARM_LENGTH = 25;
		public static final double OUTER_ARM_LENGTH = 27.25;

		// TODO: this is still incorrect, current delta between extended and retracted is 0.1809 but
		// should be 0.25
		public static final double SHOULDER_ENCODER_TO_ARM_POSITION_RATIO = 4 / 1;
		public static final double WRIST_ENCODER_TO_WRIST_POSITION_RATIO = 24 / 1;
		public static final double SHOULDER_ELBOW_GEAR_RATIO = 64 / 48;

		// PID
		// TODO: Tune PID
		public static final double ARM_K_P = 8;
		public static final double ARM_K_I = 0;
		public static final double ARM_K_D = 1;

		public static final double WRIST_DEFAULT_P = 0.1;
		public static final double WRIST_DEFAULT_I = 0;
		public static final double WRIST_DEFAULT_D = 0.5;

		// Feed Forward
		public static final double ARM_K_A = 0;
		public static final double ARM_K_G = 0.1;
		public static final double ARM_K_S = 0.1;
		public static final double ARM_K_V = 0;

		public static final double WRIST_K_A = 0;
		public static final double WRIST_K_G = 0.1;
		public static final double WRIST_K_S = 0.1;
		public static final double WRIST_K_V = 0;

		// Constraints

		public static final double MAX_ARM_ANGLE = 118;
		public static final double MIN_ARM_ANGLE = 0;
		public static final double MAX_WRIST_ANGLE = 62;
		public static final double MIN_WRIST_ANGLE = 307;

		public static final float ARM_MOTOR_TO_SHOULDER_ENCODER_RATIO = 125;
		public static final float WRIST_MOTOR_TO_WRIST_ENCODER_RATIO = 90;
		public static final float ARM_ROTATIONS_TO_SHOULDER_ENCODER_RATIO = 4;

		public static final float ARM_FORWARD_LIMIT = 156; // motor rotations
		public static final float ARM_REVERSE_LIMIT = 1;
		public static final float WRIST_FORWARD_LIMIT = 58;
		public static final float WRIST_REVERSE_LIMIT = 2;

		public static final int MIN_PERCENT_OUTPUT = -1;
		public static final int MAX_PERCENT_OUTPUT = 1;

		public static final double ARM_POS_TOLERANCE = 0.01;
		public static final double WRIST_POS_TOLERANCE = 0.01;

		public static final double ARM_VELOCITY_TOLERANCE = 0.2;
		public static final double WRIST_VELOCITY_TOLERANCE = 0.2;

		public static final double MAX_ARM_VELOCITY = 1;
		public static final double MAX_ARM_ACCELERATION =
				ARM_FORWARD_LIMIT
						/ ARM_MOTOR_TO_SHOULDER_ENCODER_RATIO
						/ ARM_ROTATIONS_TO_SHOULDER_ENCODER_RATIO; // arm rotations / second^2

		public static final double MAX_WRIST_VELOCITY = .2;
		public static final double MAX_WRIST_ACCELERATION = 0.5;

		public static final Constraints ARM_CONSTRAINTS =
				new Constraints(MAX_ARM_ACCELERATION, MAX_ARM_VELOCITY);
		public static final Constraints WRIST_CONSTRAINTS =
				new Constraints(MAX_WRIST_ACCELERATION, MAX_WRIST_VELOCITY);

		public static final double TICKS_TO_INCHES = 42 / 360; // Maybe unneeded?

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
		// 190 wrist 106 shoulder
		public static enum PositionType {
			UNKNOWN_POSITION(0, 0.30, 0.30, 0.39, 0.5),
			ARM_LOW_POSITION(0, 0.30, 0.30, 0.39, 0.5),
			ARM_MIDDLE_POSITION(0.18, 0.30, 0.30, 0.459, 0.543),
			ARM_HIGH_POSITION(0.294, 0.30, 0.30, 0.349, 0.34),
			ARM_SUBSTATION_POSITION(0.26, 0.30, 0.30, 0.5, 0.585); // ?

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
	private boolean hasResetted;
	private double armGoal;
	private double wristGoal;

	private final CANSparkMax armMotor1;
	private final CANSparkMax armMotor2;
	private final CANSparkMax wristMotor;

	private final DutyCycleEncoder shoulderEncoder;
	private final SparkMaxAbsoluteEncoder wristEncoder;

	private final ProfiledPIDController armPID;
	private final SparkMaxPIDController wristPID;

	private final ArmFeedforward wristFeedforward;

	// Network Tables

	NetworkTableInstance NTInstance;
	NetworkTable NTDevices;

	DoublePublisher shoulderEncoderPublisher;
	DoublePublisher wristEncoderPublisher;

	DoublePublisher shoulderAnglePublisher;
	DoublePublisher wristAnglePublisher;

	DoublePublisher armPIDPublisher;
	DoublePublisher armFeedforwardPublisher;

	StringPublisher armGoalPublisher;
	StringPublisher armPositionPublisher;

	BooleanPublisher armManualOverridePublisher;

	StringPublisher wristGoalPublisher;
	StringPublisher wristPositionPublisher;

	BooleanPublisher armEncoderHasResetPublisher;

	// Constructor

	public ArmSubsystem() {
		armMotor1 = new CANSparkMax(ARM_MOTOR1, MotorType.kBrushless);
		armMotor2 = new CANSparkMax(ARM_MOTOR2, MotorType.kBrushless);
		wristMotor = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);

		shoulderEncoder = new DutyCycleEncoder(SHOULDER_ENCODER_PORT_A);
		shoulderEncoder.reset();
		wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		wristEncoder.setZeroOffset(0.055);
		wristEncoder.setInverted(true);

		armPID = new ProfiledPIDController(ARM_K_P, ARM_K_I, ARM_K_D, ARM_CONSTRAINTS);
		wristPID = wristMotor.getPIDController();

		wristFeedforward = new ArmFeedforward(WRIST_K_A, WRIST_K_G, WRIST_K_S, WRIST_K_V);

		configMotors();

		currentPosition = UNKNOWN_POSITION;
		manualOverride = true;
		hasResetted = false;

		armPID.reset(getShoulderPosition());
		// wristPID.reset(getWristPosition());

		// Network Tables

		NTInstance = NetworkTableInstance.getDefault();

		NTDevices = NTInstance.getTable("Devices");

		shoulderEncoderPublisher = NTDevices.getDoubleTopic("Shoulder Encoder").publish();
		wristEncoderPublisher = NTDevices.getDoubleTopic("Wrist Encoder").publish();

		shoulderAnglePublisher = NTDevices.getDoubleTopic("Shoulder Angle").publish();
		wristAnglePublisher = NTDevices.getDoubleTopic("Wrist Angle").publish();

		armPIDPublisher = NTDevices.getDoubleTopic("Arm PID").publish();
		armFeedforwardPublisher = NTDevices.getDoubleTopic("Arm Feedforward").publish();
		armGoalPublisher = NTDevices.getStringTopic("Arm Goal").publish();
		armPositionPublisher = NTDevices.getStringTopic("Position").publish();

		wristGoalPublisher = NTDevices.getStringTopic("Wrist Goal").publish();

		armManualOverridePublisher = NTDevices.getBooleanTopic("Manual Override").publish();
		armEncoderHasResetPublisher = NTDevices.getBooleanTopic("Arm Encoder Resetted").publish();

		shoulderEncoderPublisher.set(0.0);
		wristEncoderPublisher.set(0.0);
		shoulderAnglePublisher.set(0.0);
		wristAnglePublisher.set(0.0);
		armPIDPublisher.set(0.0);
		armFeedforwardPublisher.set(0.0);

		armGoalPublisher.set(0.0 + " rotations | " + 0.0 + " degrees");
		armGoalPublisher.set(0.0 + " rotations | " + 0.0 + " degrees");
	}

	// Methods

	public void configMotors() {

		armMotor1.setInverted(false);
		armMotor1.getEncoder().setPosition(0);

		armMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
		armMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
		armMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARM_FORWARD_LIMIT);
		armMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARM_REVERSE_LIMIT);
		armMotor1.setSmartCurrentLimit(20);
		armMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);

		armMotor2.follow(armMotor1, true);
		armMotor2.setSmartCurrentLimit(20);
		armMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

		wristMotor.getEncoder().setPosition(getWristPosition() * 90);

		wristMotor.setInverted(false);
		wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
		wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
		wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, WRIST_FORWARD_LIMIT);
		wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, WRIST_REVERSE_LIMIT);
		wristMotor.setSmartCurrentLimit(20);

		setWristPID(WRIST_DEFAULT_P, WRIST_DEFAULT_I, WRIST_DEFAULT_D);
	}
	/**
	 * Sets Arm Manual Override to be on or off.
	 *
	 * @param override the value to set Manual Override as.
	 */
	public void setManualOverride(boolean override) {
		manualOverride = override;
	}

	public void setWristPID(double p, double i, double d) {
		wristPID.setP(p);
		wristPID.setI(i);
		wristPID.setD(d);
	}

	public void disableShoulderLimits() {
		armMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
		armMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
	}

	public void enableShoulderLimits() {
		armMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
		armMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
	}

	/** Resets the shoulder encoder's current position. */
	public void resetArmEncoder() {
		armMotor1.getEncoder().setPosition(0);
		shoulderEncoder.reset();
		hasResetted = true;
	}

	/**
	 * Sets the arm motor's output.
	 *
	 * @param percentOutput a value between 0-1.
	 */
	public void setArmMotor(double percentOutput) {
		percentOutput =
				MathUtil.clamp(MAX_ARM_VELOCITY * percentOutput, MIN_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT);
		armMotor1.set(percentOutput);
	}

	/**
	 * Sets the wrist motor's output.
	 *
	 * @param percentOutput a value between 0-1.
	 */
	public void setWristMotor(double percentOutput) {
		percentOutput =
				MathUtil.clamp(MAX_WRIST_VELOCITY * percentOutput, MIN_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT);
		wristPID.setReference(
				percentOutput, CANSparkMax.ControlType.kDutyCycle, 0); // calculateWristFeedforward());
	}

	/** Sets current position of the arm. Used for condition checking, nothing really else. */
	public void setPosition(PositionType position) {
		currentPosition = position;
	}

	/**
	 * Sets goal for arm PID.
	 *
	 * @param targetPos Measured in encoder rotations.
	 */
	public void setArmGoal(double targetPos) {
		armPID.setGoal(targetPos);
	}

	/**
	 * Sets goal for wrist PID.
	 *
	 * @param targetPos Measured in encoder rotations.
	 */
	public void setWristGoal(double targetPos) {
		wristPID.setReference(
				targetPos * WRIST_MOTOR_TO_WRIST_ENCODER_RATIO,
				CANSparkMax.ControlType.kPosition,
				0,
				calculateWristFeedforward());
		wristGoal = targetPos * WRIST_MOTOR_TO_WRIST_ENCODER_RATIO;
	}

	/**
	 * Uses the current target/goal of Arm PID in order to calculate output for the arm motors.
	 *
	 * @return The calculated Arm PID output.
	 */
	public double calculateArmPID() {
		return armPID.calculate(getShoulderPosition(), armPID.getGoal());
	}

	/**
	 * Calculates the arm's feedforward using the calculated angle towards center of mass.
	 *
	 * @return The calculated feedforward.
	 */
	public double calculateArmFeedforward() {
		return ARM_K_S * Math.signum(armGoal - getShoulderPosition())
				+ ARM_K_G * getAngleTowardsCenterOfMass()
				+ ARM_K_V * 0
				+ ARM_K_A * 0;
	}

	/**
	 * Uses the current target/goal of Arm PID in order to calculate output for the arm motors.
	 *
	 * @return The calculated Arm PID output./
	 */
	public double calculateWristFeedforward() {
		return wristFeedforward.calculate(
				getShoulderPosition() - getElbowPosition() + getWristPosition(), 0);
	}

	/**
	 * Gets the current shoulder encoder position.
	 *
	 * @return Current shoulder encoder position
	 */
	public double getShoulderPosition() {
		return shoulderEncoder.get() / SHOULDER_ENCODER_TO_ARM_POSITION_RATIO;
	}

	/**
	 * Gets the current elbow encoder position.
	 *
	 * @return Current elbow encoder position
	 */
	public double getElbowPosition() {
		return getShoulderPosition() * SHOULDER_ELBOW_GEAR_RATIO;
	}

	/**
	 * Gets the current wrist encoder position.
	 *
	 * @return Current wrist encoder position
	 */
	public double getWristPosition() {
		return wristEncoder.getPosition();
	}

	/**
	 * Gets current state or position of Arm Subsystem, used for extracting data from arm enum data
	 *
	 * @return The current position of the arm.
	 */
	public PositionType getPosition() {
		return currentPosition;
	}

	/**
	 * Returns whether or not arm/wrist manual override is on.
	 *
	 * @return If manual override is on.
	 */
	public boolean getManualOverride() {
		return manualOverride;
	}

	public boolean isShoulderNearLimit() {
		return (getShoulderPosition() >= ARM_FORWARD_LIMIT - 10
				|| getShoulderPosition() <= ARM_REVERSE_LIMIT + 10);
	}

	/**
	 * Converts the percentOutput into volts
	 *
	 * @param percentOutput The percentOutput to convert into volts with
	 * @return percentOutput in volts;
	 */
	public double convertToVolts(double percentOutput) {
		return percentOutput * Robot.getInstance().getVoltage();
	}

	/**
	 * Calculates the angle towards center of mass by averaging and weighing the individual arm parts
	 * (inner arm, outer arm, and intake) together.
	 *
	 * @return The calculated angle in radians.
	 */
	public double getAngleTowardsCenterOfMass() {

		// Calculate Coordinates of Centers of Mass
		double innerArmCenterOfMassX =
				INNER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT
						* Math.cos(Math.toRadians(180 - getShoulderPosition() * 360));
		double innerArmCenterOfMassY =
				INNER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT
						* Math.sin(Math.toRadians(180 - getShoulderPosition() * 360));

		double outerArmCenterOfMassX =
				OUTER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT
								* Math.cos(Math.toRadians((getElbowPosition() - getShoulderPosition()) * 360))
						+ INNER_ARM_LENGTH * Math.cos((Math.toRadians(180 - getShoulderPosition() * 360)));
		double outerArmCenterOfMassY =
				OUTER_ARM_CENTER_OF_MASS_DISTANCE_FROM_JOINT
								* Math.sin(Math.toRadians((getElbowPosition() - getShoulderPosition()) * 360))
						+ INNER_ARM_LENGTH * Math.sin(Math.toRadians(180 - getShoulderPosition() * 360));

		double intakeCenterOfMassX =
				WRIST_CENTER_OF_MASS_DISTANCE_FROM_JOINT
								* Math.cos(
										Math.toRadians(
												180 - (getWristPosition() + WRIST_CENTER_OF_MASS_ANGLE_OFFSET) * 360))
						+ OUTER_ARM_LENGTH
								* Math.cos(Math.toRadians((getElbowPosition() - getShoulderPosition()) * 360))
						+ INNER_ARM_LENGTH * Math.cos(Math.toRadians(180 - getShoulderPosition() * 360));
		double intakeCenterOfMassY =
				WRIST_CENTER_OF_MASS_DISTANCE_FROM_JOINT
								* Math.sin(
										Math.toRadians(
												180 - (getWristPosition() + WRIST_CENTER_OF_MASS_ANGLE_OFFSET) * 360))
						+ OUTER_ARM_LENGTH
								* Math.sin(Math.toRadians((getElbowPosition() - getShoulderPosition()) * 360))
						+ INNER_ARM_LENGTH * Math.sin(Math.toRadians(180 - getShoulderPosition()));

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
		if (!manualOverride && hasResetted && !currentPosition.equals(PositionType.UNKNOWN_POSITION)) {

			armMotor1.setVoltage(
					convertToVolts(
							MathUtil.clamp(
									calculateArmPID() + calculateArmFeedforward(),
									MIN_PERCENT_OUTPUT,
									MAX_PERCENT_OUTPUT)));
			System.out.println(calculateArmPID() + calculateArmFeedforward());

			wristPID.setReference(
					wristGoal, CANSparkMax.ControlType.kPosition, 0, calculateWristFeedforward());
		}

		// Network Tables

		wristEncoderPublisher.set(wristEncoder.getPosition());
		shoulderEncoderPublisher.set(getShoulderPosition());

		wristAnglePublisher.set(getWristPosition() * 360);
		shoulderAnglePublisher.set(getShoulderPosition() * 360);

		armPIDPublisher.set(calculateArmPID());
		armFeedforwardPublisher.set(calculateArmFeedforward());

		armGoalPublisher.set(
				armPID.getGoal().position
						+ " rotations | "
						+ (armPID.getGoal().position * 360)
						+ " degrees");
		wristGoalPublisher.set(wristGoal + " rotations | " + (wristGoal * 360) + " degrees");
		armPositionPublisher.set(currentPosition.toString());
		armManualOverridePublisher.set(manualOverride);

		armEncoderHasResetPublisher.set(hasResetted);
	}
}
