package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	// Constants

	public static class ArmConstants { // To find values later
		// Mech problem basically
		public static final int ARM_LENGTH = 0;
		public static final int VIRTUAL_BAR_LENGTH = 0;

		public static final int K_P = 0;
		public static final int K_I = 0;
		public static final int K_D = 0;

		// Trapezoid hee hee
		public static final int ARM_POS_TOLERANCE = 0;
		public static final int WRIST_POS_TOLERANCE = 0;

		public static final int ARM_VELOCITY_TOLERANCE = 0;
		public static final int WRIST_VELOCITY_TOLERANCE = 0;

		public static final double MAX_ARM_VELOCITY = 5;
		public static final double MAX_ARM_ACCELERATION = 5;

		public static final double MAX_WRIST_VELOCITY = 5;
		public static final double MAX_WRIST_ACCELERATION = 5;

		public static final Constraints ARM_CONSTRAINTS =
				new Constraints(MAX_ARM_ACCELERATION, MAX_ARM_VELOCITY);
		public static final Constraints WRIST_CONSTRAINTS =
				new Constraints(MAX_WRIST_ACCELERATION, MAX_WRIST_VELOCITY);

		// Arm Positions

		public static final double HIGH_NODE_ARM_ANGLE = 56.99;
		public static final double MIDDLE_NODE_ARM_ANGLE = 92;
		public static final double LOW_GRAB_ARM_ANGLE = 0;
		public static final double RETRACT_ARM_ANGLE = 169;
		public static final double SUBSTATION_ARM_ANGLE = 80;

		// find values later
		public static final double HIGH_NODE_WRIST_ANGLE = 0;
		public static final double MIDDLE_NODE_WRIST_ANGLE = 0;
		public static final double LOW_GRAB_WRIST_ANGLE = 0;
		public static final double RETRACT_WRIST_ANGLE = 90;
		public static final double SUBSTATION_WRIST_ANGLE = 0;

		// LIKELY DONT NEED
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

		public static final int ARM_ANGLE_LIMIT = 20; // to find values later
		public static final int WRIST_ANGLE_LIMIT = 20; // to find values later
	}

	// Hardware

	private final CANSparkMax armMotor;
	private final CANSparkMax wristMotor;

	private final Encoder shoulderEncoder;
	private final Encoder elbowEncoder;
	private final Encoder wristEncoder;

	private final ProfiledPIDController armPID;
	private final ProfiledPIDController wristPID;

	// Constructor

	public ArmSubsystem() {
		armMotor = new CANSparkMax(20, MotorType.kBrushless);
		wristMotor = new CANSparkMax(21, MotorType.kBrushless);
		shoulderEncoder = new Encoder(SHOULDER_ENCODER_PORT_A, SHOULDER_ENCODER_PORT_B);
		elbowEncoder = new Encoder(ELBOW_ENCODER_PORT_A, ELBOW_ENCODER_PORT_B);
		wristEncoder = new Encoder(WRIST_ENCODER_PORT_A, WRIST_ENCODER_PORT_B);

		armPID = new ProfiledPIDController(K_P, K_I, K_D, ARM_CONSTRAINTS);
		wristPID = new ProfiledPIDController(K_P, K_I, K_D, WRIST_CONSTRAINTS);

		armMotor.setIdleMode(IdleMode.kBrake);
		wristMotor.setIdleMode(IdleMode.kBrake);
		armPID.setTolerance(ARM_POS_TOLERANCE, ARM_VELOCITY_TOLERANCE);
		wristPID.setTolerance(WRIST_POS_TOLERANCE, WRIST_VELOCITY_TOLERANCE);

		elbowEncoder.reset();
		elbowEncoder.reset();
	}

	// Methods

	public double getVerticalArmPos() {
		return ((ARM_LENGTH * Math.sin(getShoulderAngle()))
				+ (VIRTUAL_BAR_LENGTH * Math.sin(getElbowAngle())));
	}

	public double getHorizontalArmPos() {
		return ((ARM_LENGTH * Math.cos(getShoulderAngle()))
				+ (VIRTUAL_BAR_LENGTH * Math.cos(getElbowAngle())));
	}

	/** Stops Arm */
	public void stopArm() {
		armMotor.stopMotor();
	}

	public void stopWrist() {
		wristMotor.stopMotor();
	}

	public void rotateArmTo(double targetAngle) {
		armPID.setGoal(targetAngle);
		armMotor.set(armPID.calculate(getShoulderAngle(), targetAngle));
	}

	public void rotateWristTo(double targetAngle) {
		wristPID.setGoal(targetAngle);
		wristMotor.set(armPID.calculate(getShoulderAngle(), targetAngle));
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

	public double getElbowAngle() {
		return elbowEncoder.getDistance();
	}

	public double getWristAngle() {
		return wristEncoder.getDistance();
	}
}
