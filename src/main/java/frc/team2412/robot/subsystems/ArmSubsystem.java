package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {

	//Constants

	public static class ArmConstants { //To find values later
		//Mech problem basically
		public static int armLength = 0;
		public static int virtualBarLength = 0;

		public static int Kp;
		public static int Ki;
		public static int Kd;

		//Arm Positions
		//Apparently, we wanted the node position to be in ticks.
		//We need an inches to ticks converter.
		public static double highCubeNodePos;
		public static double highConeNodePos;
		public static double middleCubeNodePos;
		public static double middleConeNodePos;
		public static double grabLowPos;
		public static double securePos;
        public static double substationPos;
		
		public static double ticksToInches = 42 / 360; //Maybe wrong?

		public static int armAngleLimit = 20; //to find values later
		public static int wristAngleLimit = 20; //to find values later
	}

	// Hardware

	private final CANSparkMax armMotor;
	private final CANSparkMax wristMotor;

	private final Encoder shoulderEncoder; //unsure whether or not to use WPIlib Encoder class or rev Absolute Encoder.
	private final Encoder elbowEncoder;
	private final Encoder wristEncoder;

	// Constructor

	public ArmSubsystem() {
		armMotor = new CANSparkMax(20, MotorType.kBrushless);
		wristMotor = new CANSparkMax(21, MotorType.kBrushless);
		shoulderEncoder = new Encoder(SHOULDER_ENCODER_PORT_A , SHOULDER_ENCODER_PORT_B);
		elbowEncoder = new Encoder(ELBOW_ENCODER_PORT_A, ELBOW_ENCODER_PORT_B);
		wristEncoder = new Encoder(WRIST_ENCODER_PORT_A, WRIST_ENCODER_PORT_B);

		//wristMotor.setZeroPower(CANSparkMax.BRAKE); bad?
	}

	// Methods

	//public double getAbsArmPos() {
	//return ((armLength * Math.cos(armAngle)) + virtualBarLength * Math.sin(virtualBarLength));
	//}


	public double getVerticalArmPos() {
		return ((armLength * Math.sin(getArmAngle())) + (virtualBarLength * Math.sin(getShoulderAngle())));
    }
	
	public double getHorizontalArmPos() {
		return ((armLength * Math.cos(getArmAngle())) + (virtualBarLength * Math.cos(getShoulderAngle())));
	}

	
	/** */
	public double pidFrameWork(double distance) {
		//PID values
		double initialError = distance;
		double time = 0.0;
		double error = 0.0;
		double lastTime = 0.0;
		double maxI = 0.0;
		double lastError = 0.0;
		//Actual PID loop
		while (distance - error < 10) {
			//set error value
			error = distance - error;
			//Proportional constant
			P = Kp * error;
			//Integral constant
			I += Ki * (error * (time - lastTime));

			if (I > maxI) {
				I = maxI;
			} else if (I < -maxI) {
				I = -maxI;
			}
			//Derivative constant
			D = Kd * (error - lastError) * (time / lastTime);
			//PID
			double output = P + I + D;
			//Set preverror values
			lastError = error;
			lastTime = time;
			return output;
		}
	}

	public void rotateArmTo(double targetAngle) {
		if (targetAngle > armAngleLimit) {
			return;
		}
		//rotate arm here
	}

	public void rotateHandTo(double targetAngle) {
		if (targetAngle > wristAngleLimit) {
			return;
        }
		//rotate wrist here
	}

	/** Stops Arm */
	public void stopArm() {
		armMotor.stopMotor();
	}

	/** Stops hand ඞඞඞඞඞඞඞඞඞඞඞඞ */
	public void stopWrist() {
		wristMotor.stopMotor();
	}

	public double getShoulderAngle() {
		return shoulderEncoder.getDistance(); //bad ?
	}
<<<<<<< Updated upstream
}
=======
	public double getElbowAngle() {
		return elbowEncoder.getDistance(); //bad ?
	}
	public double getWristAngle() {
		return wristEncoder.getDistance(); //bad ?
	}


	//public void driveArmToValue(double targetAngle) {
	//armMotor.changeAngle();
	//}
}
>>>>>>> Stashed changes
