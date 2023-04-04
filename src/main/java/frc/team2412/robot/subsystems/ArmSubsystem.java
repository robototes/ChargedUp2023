package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.sim.SparkMaxSimProfile.SparkMaxConstants.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.sim.PhysicsSim;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

	// CONSTANTS

	public static class ArmConstants {

		// Conversion

		public static final float ARM_MOTOR_TO_SHOULDER_ENCODER_RATIO = 75;
		public static final float WRIST_MOTOR_TO_WRIST_ENCODER_RATIO = 60.0f * 24.0f / 17.0f;
		public static final float ARM_ROTATIONS_TO_SHOULDER_ENCODER_RATIO = 4;
		public static final double SHOULDER_ENCODER_TO_ARM_POSITION_RATIO = 4 / 1;
		public static final double WRIST_ENCODER_TO_WRIST_POSITION_RATIO = 24 / 1;
		public static final double SHOULDER_TO_ELBOW_GEAR_RATIO = 64.0 / 48.0;

		// pounds/lbs
		public static final double INNER_ARM_MASS = 4.69;
		public static final double OUTER_ARM_MASS = 5.03;
		public static final double INTAKE_MASS = 5.57;

		// PID
		public static final double ARM_K_P = 4;
		public static final double ARM_K_I = 0;
		public static final double ARM_K_D = 0;

		public static final double WRIST_DEFAULT_P = 0.1;
		public static final double WRIST_DEFAULT_I = 0;
		public static final double WRIST_DEFAULT_D = 0.5;

		// Feed Forward
		public static final double ARM_K_S = 0.005;
		public static final double ARM_K_V = 0.0;
		public static final double ARM_K_A = 0.0;
		public static final double ARM_K_G = 0.1;

		public static final double WRIST_K_S = 0.1;
		public static final double WRIST_K_V = 0.0;
		public static final double WRIST_K_A = 0.0;
		public static final double WRIST_K_G = 0.1;

		// Constraints

		public static final int MIN_PERCENT_OUTPUT = -1;
		public static final int MAX_PERCENT_OUTPUT = 1;

		public static final double MAX_ARM_ANGLE = 118;
		public static final double MIN_ARM_ANGLE = 6;
		public static final double MAX_WRIST_ANGLE = 62;
		public static final double MIN_WRIST_ANGLE = 307;

		public static final float ARM_FORWARD_LIMIT = 93.8f; // motor rotations
		public static final float ARM_REVERSE_LIMIT = 2;
		public static final float WRIST_FORWARD_LIMIT = 62;
		public static final float WRIST_REVERSE_LIMIT = 2;
		public static final float WRIST_ENCODER_OFFSET = -0.20f;

		public static final double ARM_VELOCITY_TOLERANCE = 0.2;
		public static final double WRIST_VELOCITY_TOLERANCE = 0.2;

		// Trapezoid Profile
		public static final double MAX_ARM_VELOCITY = 1;
		public static final double MAX_ARM_ACCELERATION = 1;

		public static final Constraints ARM_CONSTRAINTS =
				new Constraints(MAX_ARM_VELOCITY, MAX_ARM_ACCELERATION);

		public static final double MAX_WRIST_VELOCITY = 0.5;
		public static final double MAX_WRIST_ACCELERATION = 0.5;

		public static final Constraints WRIST_CONSTRAINTS =
				new Constraints(MAX_WRIST_VELOCITY, MAX_WRIST_ACCELERATION);

		public static final double ARM_POS_ADJUST_SENSITIVITY_DEFAULT = 0.007;
		public static final double WRIST_POS_ADJUST_SENSITIVITY_DEFAULT = 0.01;

		// ENUM

		/*
		 * TODO: Prescore presets
		 *
		 * Find arm and wrist angles (see onenote page)
		 *
		 * Values are angles that correspond to specific arm positions:
		 * Arm Angle
		 * Retracted Wrist Angle
		 * Prescore Wrist Angle
		 * Scoring Wrist Angle
		 */
		public static enum PositionType {
			UNKNOWN_POSITION(0.212, 0.08, 0.467, 0.467),
			ARM_LOW_POSITION(0.212, 0.08, 0.46, 0.453),
			ARM_MIDDLE_POSITION(0.415, 0.08, 0.42, 0.5),
			ARM_HIGH_POSITION(0.6546, 0.08, 0.465, 0.473),
			ARM_SUBSTATION_POSITION(0.6, 0.08, 0.56, 0.56); // ?

			public final double armAngle;
			public final double retractedWristAngle;
			public final double prescoringWristAngle;
			public final double scoringWristAngle;

			PositionType(
					double armAngle,
					double retractedWristAngle,
					double prescoringWristAngle,
					double scoringWristAngle) {
				this.armAngle = armAngle;
				this.retractedWristAngle = retractedWristAngle;
				this.prescoringWristAngle = prescoringWristAngle;
				this.scoringWristAngle = scoringWristAngle;
			}
		}
	}

	// VARIABLES

	private PositionType currentPosition = PositionType.UNKNOWN_POSITION;
	private boolean manualOverride = false;
	private double armGoal = PositionType.UNKNOWN_POSITION.armAngle;
	private double wristGoal =
			PositionType.UNKNOWN_POSITION.retractedWristAngle * WRIST_MOTOR_TO_WRIST_ENCODER_RATIO;

	// Joysticks (for fine tune control in preset mode)

	private DoubleSupplier armPosAdjustJoystick;
	private DoubleSupplier wristPosAdjustJoystick;

	// Hardware
	private final CANSparkMax armMotor1;
	private final CANSparkMax armMotor2;
	private final CANSparkMax wristMotor;

	private final SparkMaxAbsoluteEncoder elbowEncoder;
	private final SparkMaxAbsoluteEncoder wristEncoder;

	private final ProfiledPIDController armPID;
	private final SparkMaxPIDController wristPID;

	private final ArmFeedforward wristFeedforward;

	// Network Tables
	NetworkTableInstance NTInstance;
	NetworkTable NTDevices;

	DoublePublisher shoulderPositionPublisher;
	DoublePublisher elbowPositionPublisher;
	DoublePublisher wristPositionPublisher;

	DoublePublisher armPIDPublisher;
	DoublePublisher armFeedforwardPublisher;

	DoublePublisher armGoalPublisher;
	StringPublisher armPositionPublisher;

	BooleanPublisher armManualOverridePublisher;

	DoublePublisher wristGoalPublisher;

	DoubleSubscriber armVoltageOverride;
	BooleanSubscriber armVoltageOverrideEnable;
	DoublePublisher armVoltagePublisher;
	DoublePublisher armOutputPublisher;
	DoublePublisher armCurrentPublisher;
	DoublePublisher armLastSetValuePublisher;

	GenericEntry armPosAdjustSensitivity =
			Shuffleboard.getTab("Arm")
					.add("Arm Adjust Sensitivity", ARM_POS_ADJUST_SENSITIVITY_DEFAULT)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 0.025))
					.getEntry();
	GenericEntry wristPosAdjustSensitivity =
			Shuffleboard.getTab("Arm")
					.add("Wrist Adjust Sensitivity", WRIST_POS_ADJUST_SENSITIVITY_DEFAULT)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 0.025))
					.getEntry();

	// CONSTRUCTOR

	public ArmSubsystem() {
		armMotor1 = new CANSparkMax(ARM_MOTOR_1, MotorType.kBrushless);
		armMotor2 = new CANSparkMax(ARM_MOTOR_2, MotorType.kBrushless);
		wristMotor = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);

		elbowEncoder = armMotor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		wristEncoder.setZeroOffset(0);
		wristEncoder.setInverted(true);
		elbowEncoder.setInverted(true);

		armPID = new ProfiledPIDController(ARM_K_P, ARM_K_I, ARM_K_D, ARM_CONSTRAINTS);
		wristPID = wristMotor.getPIDController();

		wristFeedforward = new ArmFeedforward(WRIST_K_A, WRIST_K_G, WRIST_K_S, WRIST_K_V);

		configMotors();

		armPID.reset(getShoulderPosition());

		// Network Tables

		NTInstance = NetworkTableInstance.getDefault();

		NTDevices = NTInstance.getTable("Devices");

		armPositionPublisher = NTDevices.getStringTopic("Position").publish();
		shoulderPositionPublisher = NTDevices.getDoubleTopic("Shoulder Pos").publish();
		elbowPositionPublisher = NTDevices.getDoubleTopic("Elbow Pos").publish();
		wristPositionPublisher = NTDevices.getDoubleTopic("Wrist Pos").publish();
		armPIDPublisher = NTDevices.getDoubleTopic("Arm PID").publish();
		armFeedforwardPublisher = NTDevices.getDoubleTopic("Arm Feedforward").publish();
		armGoalPublisher = NTDevices.getDoubleTopic("Arm Goal").publish();
		armVoltagePublisher = NTDevices.getDoubleTopic("Arm voltage").publish();
		armOutputPublisher = NTDevices.getDoubleTopic("Arm output").publish();
		armCurrentPublisher = NTDevices.getDoubleTopic("Arm current").publish();
		armLastSetValuePublisher = NTDevices.getDoubleTopic("Arm last set value").publish();
		wristGoalPublisher = NTDevices.getDoubleTopic("Wrist Goal").publish();
		armManualOverridePublisher = NTDevices.getBooleanTopic("Manual Override").publish();

		shoulderPositionPublisher.set(0.0);
		elbowPositionPublisher.set(0.0);
		wristPositionPublisher.set(0.0);
		armGoalPublisher.set(0.0);
		armPIDPublisher.set(0.0);
		armFeedforwardPublisher.set(0.0);
		armVoltagePublisher.set(0.0);
		armOutputPublisher.set(0.0);
		armCurrentPublisher.set(0.0);
		armLastSetValuePublisher.set(0.0);
	}

	// METHODS

	public void configMotors() {

		armMotor1.setInverted(false);
		armMotor1.getEncoder().setPosition(0);

		armMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
		armMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
		armMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARM_FORWARD_LIMIT);
		armMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARM_REVERSE_LIMIT);
		armMotor1.setSmartCurrentLimit(20);
		armMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		armMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

		armMotor2.follow(armMotor1, true);
		armMotor2.setSmartCurrentLimit(20);
		armMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

		wristMotor.getEncoder().setPosition(getWristPosition() * WRIST_MOTOR_TO_WRIST_ENCODER_RATIO);

		wristMotor.setInverted(false);
		wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
		wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
		wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, WRIST_FORWARD_LIMIT);
		wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, WRIST_REVERSE_LIMIT);
		wristMotor.setSmartCurrentLimit(25);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		setWristPID(WRIST_DEFAULT_P, WRIST_DEFAULT_I, WRIST_DEFAULT_D);

		armMotor1.burnFlash();
		armMotor2.burnFlash();
		wristMotor.burnFlash();
	}

	public void setPresetAdjustJoysticks(
			DoubleSupplier armPosAdjustJoystick, DoubleSupplier wristPosAdjustJoystick) {
		this.armPosAdjustJoystick = armPosAdjustJoystick;
		this.wristPosAdjustJoystick = wristPosAdjustJoystick;
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

	public void simInit(PhysicsSim sim) {
		sim.addSparkMax(armMotor1, STALL_TORQUE, FREE_SPEED_RPM);
		sim.addSparkMax(armMotor2, STALL_TORQUE, FREE_SPEED_RPM);

		sim.addSparkMax(wristMotor, STALL_TORQUE, FREE_SPEED_RPM);
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
		armLastSetValuePublisher.set(percentOutput);
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

		return armPID.calculate(getElbowPosition(), armPID.getGoal());
	}

	/**
	 * Calculates the arm's feedforward using the calculated angle towards center of mass.
	 *
	 * @return The calculated feedforward.
	 */
	public double calculateArmFeedforward() {
		// The values below are from our maximum and minimum elbow positions line of best fit for feed
		// forward values
		// -0.111639604	0.064541504
		double elbowPosition = getElbowPosition();

		return ARM_K_S * Math.signum(armGoal - elbowPosition) + elbowPosition <= 0.48
				? 0.0
				: (elbowPosition * -0.111639604 + 0.064541504);
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
	 * Gets current state or position of Arm Subsystem, used for extracting data from arm enum data
	 *
	 * @return The current position of the arm.
	 */
	public PositionType getPosition() {
		return currentPosition;
	}

	/**
	 * Gets the current shoulder encoder position.
	 *
	 * @return Current shoulder encoder position
	 */
	public double getShoulderPosition() {
		return getElbowPosition() / SHOULDER_TO_ELBOW_GEAR_RATIO;
	}

	/**
	 * Gets the current elbow encoder position.
	 *
	 * @return Current elbow encoder position
	 */
	public double getElbowPosition() {
		return elbowEncoder.getPosition();
	}

	/**
	 * Gets the current wrist encoder position.
	 *
	 * @return Current wrist encoder position
	 */
	public double getWristPosition() {
		return Math.max(wristEncoder.getPosition() + WRIST_ENCODER_OFFSET, 0);
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

	public double rotationsToRadians(double rotations) {
		return rotations * 2 * Math.PI;
	}

	/**
	 * Returns whether or not arm/wrist manual override is on.
	 *
	 * @return If manual override is on.
	 */
	public boolean getManualOverride() {
		return manualOverride;
	}

	public boolean isArmNearGoal(double nearValue) {
		return Math.abs(getElbowPosition() - armPID.getGoal().position) <= nearValue;
	}

	/** Updates the arm motor's output based off of the wrist's goal */
	public void updateArmMotorOutput() {

		double percentOutput =
				MathUtil.clamp(
						calculateArmPID() + calculateArmFeedforward(), MIN_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT);
		double voltage = convertToVolts(percentOutput);

		armMotor1.setVoltage(voltage);
	}

	/** Updates the wrist motor's output based off of the wrist's goal */
	public void updateWristMotorOutput() {
		wristPID.setReference(
				wristGoal, CANSparkMax.ControlType.kPosition, 0, calculateWristFeedforward());
	}

	@Override
	public void periodic() {
		// Periodic Arm movement for Preset Angle Control
		if (!manualOverride && armPosAdjustJoystick != null && wristPosAdjustJoystick != null) {
			// fine tune adjust presets
			double armPosAdjust =
					-MathUtil.applyDeadband(armPosAdjustJoystick.getAsDouble(), 0.05)
							* armPosAdjustSensitivity.getDouble(ARM_POS_ADJUST_SENSITIVITY_DEFAULT);
			double wristPosAdjust =
					MathUtil.applyDeadband(wristPosAdjustJoystick.getAsDouble(), 0.05)
							* wristPosAdjustSensitivity.getDouble(WRIST_POS_ADJUST_SENSITIVITY_DEFAULT);

			setArmGoal(armPID.getGoal().position + armPosAdjust);
			setWristGoal((wristGoal / WRIST_MOTOR_TO_WRIST_ENCODER_RATIO) + wristPosAdjust);
		}
		if (!manualOverride && !currentPosition.equals(PositionType.UNKNOWN_POSITION)) {
			updateArmMotorOutput();
			updateWristMotorOutput();
		}

		// Network Tables

		shoulderPositionPublisher.set(getShoulderPosition());
		elbowPositionPublisher.set(getElbowPosition());
		wristPositionPublisher.set(getWristPosition());

		armGoalPublisher.set(armPID.getGoal().position);
		wristGoalPublisher.set(wristGoal / WRIST_MOTOR_TO_WRIST_ENCODER_RATIO);

		armPIDPublisher.set(calculateArmPID());

		armFeedforwardPublisher.set(calculateArmFeedforward());

		armPositionPublisher.set(currentPosition.toString());
		armManualOverridePublisher.set(manualOverride);

		armVoltagePublisher.set(armMotor1.get());
		armOutputPublisher.set(armMotor1.getAppliedOutput());
		armCurrentPublisher.set(armMotor1.getOutputCurrent());
	}
}
