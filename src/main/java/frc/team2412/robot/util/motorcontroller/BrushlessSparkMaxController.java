package frc.team2412.robot.util.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;

public class BrushlessSparkMaxController extends MotorController {
	private final CANSparkMax motor;
	private final SparkMaxPIDController motorPID;

	private MotorControlMode mode;

	public BrushlessSparkMaxController(int id, MotorControlMode mode) {
		this.motor = new CANSparkMax(id, MotorType.kBrushless);
		this.motorPID = motor.getPIDController();
		this.mode = mode;

		useIntegratedEncoder();
	}

	public BrushlessSparkMaxController(int id) {
		this(id, MotorControlMode.PERCENT);
	}

	@Override
	public void setNeutralMode(MotorNeutralMode mode) {
		this.motor.setIdleMode(mode.getREV());
	}

	@Override
	public void configFactoryDefault() {
		this.motor.restoreFactoryDefaults();
	}

	@Override
	public void setPID(double P, double I, double D) {
		motorPID.setP(P);
		motorPID.setI(I);
		motorPID.setD(D);
	}

	@Override
	public void set(double setpoint) {
		set(setpoint, mode);
	}

	@Override
	public void set(double setpoint, MotorControlMode mode) {
		if (mode == MotorControlMode.VELOCITY) {
			setpoint = setpoint * 60; // rps to rpm
		}
		if (mode == MotorControlMode.VOLTAGE) {
			this.motor.setVoltage(setpoint);
			return;
		}
		motorPID.setReference(setpoint, mode.getREV());
	}

	@Override
	public void setIntegratedEncoderPosition(double position) {
		motor.getEncoder().setPosition(position);
	}

	@Override
	public double getIntegratedEncoderPosition() {
		return motor.getEncoder().getPosition();
	}

	@Override
	public void setControlMode(MotorControlMode mode) {
		this.mode = mode;
	}

	public void setFeedbackDevice(MotorFeedbackSensor sensor) {
		motorPID.setFeedbackDevice(sensor);
	}

	@Override
	public void useIntegratedEncoder() {
		setFeedbackDevice(motor.getEncoder());
	}

	@Override
	public void setInverted(boolean invert) {
		motor.setInverted(invert);
	}

	@Override
	public void setNominalVoltage(double voltage) {
		motor.enableVoltageCompensation(voltage);
	}
}
