package frc.team2412.robot.util.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.team2412.robot.Hardware;

public class TalonFXController extends MotorController {
	private final WPI_TalonFX motor;
	private final int motorPIDIndex = 0;

	private ControlMode mode;

	public TalonFXController(int id, ControlMode mode) {
		this.motor = new WPI_TalonFX(id);
		this.mode = mode;
		motor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, motorPIDIndex, Hardware.CAN_TIMEOUT_MS);
	}

	public TalonFXController(int id) {
		this(id, ControlMode.PercentOutput);
	}

	@Override
	public void setNeutralMode(MotorNeutralMode mode) {
		motor.setNeutralMode(mode.getCTRE());
	}

	@Override
	public void configFactoryDefault() {
		motor.configFactoryDefault();
	}

	@Override
	public void setEncoderInverted(boolean inverted) {
		motor.setSensorPhase(inverted);
	}

	@Override
	public void setPID(double P, double I, double D) {
		motor.config_kP(motorPIDIndex, P);
		motor.config_kI(motorPIDIndex, I);
		motor.config_kD(motorPIDIndex, D);
	}

	@Override
	public void set(double setpoint) {
		motor.set(mode, setpoint);
	}

	@Override
	public void set(double setpoint, MotorControlMode mode) {
		motor.set(mode.getCTRE(), setpoint);
	}

	@Override
	public void setIntegratedEncoderPosition(double position) {
		motor.setSelectedSensorPosition(position);
	}

	@Override
	public double getIntegratedEncoderPosition() {
		return motor.getSelectedSensorPosition();
	}

	@Override
	public void setControlMode(MotorControlMode mode) {
		this.mode = mode.getCTRE();
	}
}
