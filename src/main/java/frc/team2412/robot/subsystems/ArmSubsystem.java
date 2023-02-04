package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	// Constants

	public static class ArmConstants { // To find values later
		// Mech problem basically

		public static final double MAX_ARM_SPEED = 0.35;
		public static final double MAX_WRIST_SPEED = 0.3;
		public static final double ARM_LENGTH = 0;
		public static final double VIRTUAL_BAR_LENGTH = 0;

		// PID
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

		// Arm Positions

		// TODO: maybe?
		// Apparently, we wanted the node position to be in ticks.
		// We need an inches to ticks converter.
		public static final double HIGH_NODE_CUBE_POS = 0;
		public static final double HIGH_NODE_CONE_POS = 0;
		public static final double MIDDLE_NODE_CUBE_POS = 0;
		public static final double MIDDLE_NODE_CONE_POS = 0;
		public static final double GRAB_LOW_POS = 0;
		public static final double RETRACT_POS = 0;
		public static final double SUBSTATION_POS = 0;

		public static final double TICKS_TO_INCHES = 42 / 360; // Maybe wrong?

		public static final double MIN_ARM_ANGLE = 56.9;
		public static final double MAX_ARM_ANGLE = 170; // to find values later
		public static final double WRIST_ANGLE_LIMIT = 20; // to find values later

		// ENUM :(

		/*
		 * TODO:
		 * Find arm and wrist angles (see onenote page)
		 */
		public static enum PositionType {
			LOW(0, 0, 279.6, 0, 0),
			MIDDLE(87.42, 0, 279.6, 0, 80.27),
			HIGH(50, 0, 279.6, 0, 111),
			SUBSTATION(80, 0, 279.6, 0, 0); // ?

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

	private final CANSparkMax armMotor;
	private final CANSparkMax wristMotor;

	private final Encoder shoulderEncoder;
	private final Encoder wristEncoder;

	private final ProfiledPIDController armPID;
	private final ProfiledPIDController wristPID;

	private final ArmFeedforward armFeedforward;
	private final ArmFeedforward wristFeedforward;

	// Constructor

	public ArmSubsystem() {
		armMotor = new CANSparkMax(ARM_MOTOR, MotorType.kBrushless);
		wristMotor = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);

		shoulderEncoder = new Encoder(SHOULDER_ENCODER_PORT_A, SHOULDER_ENCODER_PORT_B);
		wristEncoder = new Encoder(WRIST_ENCODER_PORT_A, WRIST_ENCODER_PORT_B);

		armPID = new ProfiledPIDController(ARM_K_P, ARM_K_I, ARM_K_D, ARM_CONSTRAINTS);
		wristPID = new ProfiledPIDController(WRIST_K_P, WRIST_K_I, WRIST_K_D, WRIST_CONSTRAINTS);

		armFeedforward = new ArmFeedforward(ARM_K_A, ARM_K_G, ARM_K_S, ARM_K_V);
		wristFeedforward = new ArmFeedforward(WRIST_K_A, WRIST_K_G, WRIST_K_S, WRIST_K_V);

		armMotor.setIdleMode(IdleMode.kBrake);
		wristMotor.setIdleMode(IdleMode.kBrake);
		armPID.setTolerance(ARM_POS_TOLERANCE, ARM_VELOCITY_TOLERANCE);
		wristPID.setTolerance(WRIST_POS_TOLERANCE, WRIST_VELOCITY_TOLERANCE);

		currentPosition = LOW;
		manualOverride = false;

		shoulderEncoder.reset();
		wristEncoder.reset();
	}

	// Methods

	public void setArmMotor(double percentOutput) {
		armMotor.set(MAX_ARM_SPEED * percentOutput);
	}

	public void setWristMotor(double percentOutput) {
		wristMotor.set(MAX_WRIST_SPEED + percentOutput);
	}

	public void setPosition(PositionType position) {
		currentPosition = position;
	}

	public void setManualOverride(boolean override) {
		manualOverride = override;
	}

	public void setArmGoal(double targetAngle) {
		armPID.setGoal(targetAngle);
	}

	public void setWristGoal(double targetAngle) {
		wristPID.setGoal(targetAngle);
	}

	public double calculateArmPID() {
		return armPID.calculate(shoulderEncoder.getDistance(), armPID.getGoal());
	}

	public double calculateWristPID() {
		return armPID.calculate(wristEncoder.getDistance(), wristPID.getGoal());
	}

	public double calculatearmFeedforward() {
		return armFeedforward.calculate(shoulderEncoder.getDistance(), shoulderEncoder.getRate());
	}

	public double calculatewristFeedforward() {
		return wristFeedforward.calculate(
				wristEncoder.getDistance(), wristEncoder.getRate()); // getRate = getVelocity?
	}

	public boolean armIsAtGoal() {
		return armPID.atGoal();
	}

	public boolean wristIsAtGoal() {
		return wristPID.atGoal();
	}

	public double getShoulderAngle() {
		return shoulderEncoder.getDistance();
	}

	public double getWristAngle() {
		return wristEncoder.getDistance();
	}

	public PositionType getPosition() {
		return currentPosition;
	}

	/* TODO:
	 * FeedForward for Arm and Wrist
	 */
	@Override
	public void periodic() {
		if (!manualOverride) {
			armMotor.setVoltage(calculateArmPID() + calculatearmFeedforward());
			wristMotor.setVoltage(calculateWristPID() + calculatewristFeedforward());
		}
	}
}
