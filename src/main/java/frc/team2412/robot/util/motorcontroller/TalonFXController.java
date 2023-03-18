package frc.team2412.robot.util.motorcontroller;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.sim.PhysicsSim;

public class TalonFXController extends MotorController {
	private static final double TICKS_PER_ROTATION = 2048.0;

	private final WPI_TalonFX motor;
	private final int motorPIDIndex = 0;

	private MotorControlMode mode;

	public TalonFXController(int id, MotorControlMode mode) {
		this.motor = new WPI_TalonFX(id);
		this.mode = mode;
		motor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, motorPIDIndex, Hardware.CAN_TIMEOUT_MS);
	}

	public TalonFXController(int id) {
		this(id, MotorControlMode.PERCENT);
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
	public void setPID(double P, double I, double D) {
		motor.config_kP(motorPIDIndex, P);
		motor.config_kI(motorPIDIndex, I);
		motor.config_kD(motorPIDIndex, D);
	}

	@Override
	public void set(double setpoint) {
		set(setpoint, mode);
	}

	@Override
	public void set(double setpoint, MotorControlMode mode) {
		// convert to ticks from rotations
		if (mode == MotorControlMode.POSITION || mode == MotorControlMode.VELOCITY) {
			setpoint *= TICKS_PER_ROTATION;
		}

		if (mode == MotorControlMode.VELOCITY) {
			setpoint = setpoint / 10; // seconds to 100ms
		}

		if (mode == MotorControlMode.VOLTAGE) {
			motor.setVoltage(setpoint);
		} else {
			motor.set(mode.getCTRE(), setpoint);
		}
	}

	@Override
	public void setIntegratedEncoderPosition(double position) {
		// convert from rotations to ticks
		motor.setSelectedSensorPosition(position * TICKS_PER_ROTATION);
	}

	@Override
	public double getIntegratedEncoderPosition() {
		// convert from ticks to rotations
		return motor.getSelectedSensorPosition() / TICKS_PER_ROTATION;
	}

	@Override
	public void setControlMode(MotorControlMode mode) {
		this.mode = mode;
	}

	// talonfx does not need this but neo does
	@Override
	public void useIntegratedEncoder() {}

	@Override
	public void setInverted(boolean inverted) {
		motor.setInverted(inverted);
	}

	@Override
	public void setNominalVoltage(double voltage) {
		motor.enableVoltageCompensation(true);
		motor.configVoltageCompSaturation(voltage);
	}

	@Override
	public void configureOptimization() {
		// nothing to do here for CTRE
	}

	@Override
	public double getVelocity() {
		return motor.getSelectedSensorVelocity();
	}

	@Override
	public void setMeasurementPeriod(int periodMS) {
		// nothing hehe
	}

	@Override
	public void simulationConfig(PhysicsSim sim) {
		sim.addTalonFX(motor, 2, 2000, false);
	}

}
