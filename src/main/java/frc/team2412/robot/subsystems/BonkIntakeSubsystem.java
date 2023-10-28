package frc.team2412.robot.subsystems;

import static frc.team2412.robot.sim.SparkMaxSimProfile.SparkMaxConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.sim.PhysicsSim;
import java.util.function.DoubleSupplier;

public class BonkIntakeSubsystem extends SubsystemBase {

	// constants
	public static final double INTAKE_SPEED = 0.2;
	public static final double INTAKE_FAST_SPEED = 1.0;

	public static final double WRIST_PID_P = 0.1;
	public static final double WRIST_PID_I = 0.0;
	public static final double WRIST_PID_D = 0.5;
	public static final double WRIST_PID_FF = 0.0;
	public static final double WRIST_MIN_OUTPUT = -1.0;
	public static final double WRIST_MAX_OUTPUT = 1.0;
	public static final float WRIST_FORWARD_LIMIT = 38f;
	public static final float WRIST_REVERSE_LIMIT = 0.0f;

	// motors
	private final CANSparkMax wristMotor;
	private final CANSparkMax intakeMotor1;
	private final CANSparkMax intakeMotor2;

	private final SparkMaxPIDController wristPIDController;
	private final RelativeEncoder wristEncoder;

	private double wristGoal;

	public BonkIntakeSubsystem() {
		wristMotor = new CANSparkMax(Hardware.BONK_INTAKE_WRIST_MOTOR, MotorType.kBrushless);
		intakeMotor1 = new CANSparkMax(Hardware.BONK_INTAKE_MOTOR_1, MotorType.kBrushless);
		intakeMotor2 = new CANSparkMax(Hardware.BONK_INTAKE_MOTOR_2, MotorType.kBrushless);

		wristEncoder = wristMotor.getEncoder();
		wristPIDController = wristMotor.getPIDController();

		resetMotors();

		// config pid for wrist motor

		updateNetworkTables();
	}

	/** Configure and reset motors */
	private void resetMotors() {
		wristMotor.restoreFactoryDefaults();
		intakeMotor1.restoreFactoryDefaults();
		intakeMotor2.restoreFactoryDefaults();

		wristMotor.setIdleMode(IdleMode.kCoast);
		intakeMotor1.setIdleMode(IdleMode.kBrake);
		intakeMotor2.setIdleMode(IdleMode.kBrake);

		wristMotor.setSmartCurrentLimit(40);
		intakeMotor1.setSmartCurrentLimit(40);
		intakeMotor2.setSmartCurrentLimit(40);

		wristEncoder.setPosition(0);
		wristGoal = 0;

		wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
		wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
		wristMotor.setSoftLimit(SoftLimitDirection.kForward, WRIST_FORWARD_LIMIT);
		wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WRIST_REVERSE_LIMIT);

		// intakeMotor2.follow(intakeMotor1);

		wristMotor.setInverted(true);
		intakeMotor1.setInverted(false);
		intakeMotor2.setInverted(true);

		wristPIDController.setP(WRIST_PID_P, 0);
		wristPIDController.setI(WRIST_PID_I, 0);
		wristPIDController.setD(WRIST_PID_D, 0);
		wristPIDController.setFF(WRIST_PID_FF, 0);
		wristPIDController.setOutputRange(WRIST_MIN_OUTPUT, WRIST_MAX_OUTPUT, 0);

		wristMotor.burnFlash();
		intakeMotor1.burnFlash();
		intakeMotor2.burnFlash();
	}

	private void updateNetworkTables() {
		SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
		SmartDashboard.putNumber("Wrist Goal", wristGoal);
	}

	// intake stuff:

	/**
	 * Set the intake motors to a given speed
	 *
	 * @param speed The speed to set the motors to
	 */
	private void setIntakeSpeed(double speed) {
		intakeMotor1.set(speed);
		intakeMotor2.set(speed);
	}

	public CommandBase intakeInCommand() {
		return this.runOnce(() -> setIntakeSpeed(INTAKE_SPEED));
	}

	public CommandBase intakeOutCommand() {
		return this.runOnce(() -> setIntakeSpeed(-INTAKE_SPEED));
	}

	public CommandBase intakeFastOutCommand() {
		return this.runOnce(() -> setIntakeSpeed(-INTAKE_FAST_SPEED));
	}

	public CommandBase intakeStopCommand() {
		return this.runOnce(() -> setIntakeSpeed(0));
	}

	// wrist stuff:

	public void setWristGoal(double goal) {
		wristGoal = goal;

		wristPIDController.setReference(wristGoal, CANSparkMax.ControlType.kPosition, 0);
	}

	public CommandBase adjustWristCommand(DoubleSupplier adjustment) {
		return this.run(
				() -> {
					if (wristGoal + adjustment.getAsDouble() > WRIST_FORWARD_LIMIT)
						setWristGoal(WRIST_FORWARD_LIMIT);
					else if (wristGoal + adjustment.getAsDouble() < WRIST_REVERSE_LIMIT)
						setWristGoal(WRIST_REVERSE_LIMIT);
					else setWristGoal(wristGoal + adjustment.getAsDouble());
				});
	}

	public void simInit(PhysicsSim sim) {
		sim.addSparkMax(wristMotor, STALL_TORQUE, FREE_SPEED_RPM);
		sim.addSparkMax(intakeMotor1, STALL_TORQUE, FREE_SPEED_RPM);
		sim.addSparkMax(intakeMotor2, STALL_TORQUE, FREE_SPEED_RPM);
	}

	@Override
	public void periodic() {
		updateNetworkTables();
	}
}
