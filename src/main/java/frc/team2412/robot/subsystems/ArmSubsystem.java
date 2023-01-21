package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {

	// Constants

	public static class ArmConstants { // To find values later
		// Mech problem basically
		public static int armLength = 0;
		public static int virtualBarLength = 0;

		public static int Kp;
		public static int Ki;
		public static int Kd;

		public static double nodePos; // Apparently, we wanted the node position to be in ticks.
		// We need an inches to ticks converter.
		public static double ticksToInches = 42 / 360; // placeholder not real unfortunately :(
	}

	// Hardware

	private final CANSparkMax armMotor;
	private final CANSparkMax wristMotor;

	private final Encoder
			shoulderEncoder; // unsure whether or not to use WPIlib Encoder class or rev Absolute Encoder.
	private final Encoder elbowEncoder;
	private final Encoder wristEncoder;

	// Constructor
	public ArmSubsystem() {
		armMotor = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
		wristMotor = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);
		shoulderEncoder = new Encoder();
		elbowEncoder = new Encoder();
		wristEncoder = new Encoder();

		wristMotor.setZeroPower(CANSparkMax.BRAKE);
	}

	// Methods

	public double getAbsArmPos() {
		return ((armLength * Math.cos(armAngle)) + virtualBarLength * Math.sin(virtualBarLength));
	}

	/** */
	public double pidFrameWork(double distance) {
		// PID values
		double initialError = distance;
		double time = 0.0;
		double error = 0.0;
		double lastTime = 0.0;
		double maxI = 0.0;
		double lastError = 0.0;
		// Actual PID loop
		while (distance - error < 10) {
			// set error value
			error = distance - error;
			// Proportional constant
			P = Kp * error;
			// Integral constant
			I += Ki * (error * (time - lastTime));

			if (I > maxI) {
				I = maxI;
			} else if (I < -maxI) {
				I = -maxI;
			}
			// Derivative constant
			D = Kd * (error - lastError) * (time / lastTime);
			// PID
			double output = P + I + D;
			// Set preverror values
			lastError = error;
			lastTime = time;
			return output;
		}
	}

	public void rotateArmTo(double angle) {
		armMotor.rotateArm();
	}

	public void rotateHandTo(double angle) {
		handMotor.rotateHand();
	}

	/** Stops Arm */
	public void stopArm() {
		armMotor.stopMotor();
	}

	/** Stops hand ඞඞඞඞඞඞඞඞඞඞඞඞ */
	public void stopHand() {
		handMotor.stopMotor();
	}

	public void driveArmToValue(double angle) {
		armMotor.changeAngle();
	}
}
