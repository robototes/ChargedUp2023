package frc.team2412.robot.util.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.team2412.robot.sim.PhysicsSim;

public abstract class MotorController {
	public enum MotorNeutralMode {
		COAST(NeutralMode.Coast, IdleMode.kCoast),
		BRAKE(NeutralMode.Brake, IdleMode.kBrake);

		private final NeutralMode ctre;
		private final IdleMode rev;

		MotorNeutralMode(NeutralMode ctre, IdleMode rev) {
			this.ctre = ctre;
			this.rev = rev;
		}

		public NeutralMode getCTRE() {
			return ctre;
		}

		public IdleMode getREV() {
			return rev;
		}
	}

	public enum MotorControlMode {
		POSITION(ControlMode.Position, ControlType.kPosition),
		VELOCITY(ControlMode.Velocity, ControlType.kVelocity),
		// CTRE does not have a voltage control mode, we just set it with motor.setVoltage()
		VOLTAGE(null, ControlType.kVoltage),
		PERCENT(ControlMode.PercentOutput, ControlType.kDutyCycle);

		private final ControlMode ctre;
		private final ControlType rev;

		MotorControlMode(ControlMode ctre, ControlType rev) {
			this.ctre = ctre;
			this.rev = rev;
		}

		public ControlMode getCTRE() {
			return ctre;
		}

		public ControlType getREV() {
			return rev;
		}
	}

	public abstract void setNeutralMode(MotorNeutralMode mode);

	public abstract void configFactoryDefault();

	public abstract void setPIDF(double P, double I, double D, double F);

	/** Units: rotations for position or velocity, voltage for voltage, percent for percent */
	public abstract void set(double setpoint);

	/** Units: rotations for position or velocity, voltage for voltage, percent for percent */
	public abstract void set(double setpoint, MotorControlMode mode);

	public abstract void stop();

	/** Units: rotations */
	public abstract void setIntegratedEncoderPosition(double position);

	/** Units: rotations */
	public abstract double getIntegratedEncoderPosition();

	public abstract void setControlMode(MotorControlMode mode);

	public abstract void useIntegratedEncoder();

	public abstract void setInverted(boolean inverted);

	public abstract void setNominalVoltage(double voltage);

	public abstract void configureOptimization();

	public abstract void setAverageDepth(int depth);

	/** Units: rotations per second */
	public abstract double getVelocity();

	public abstract double getPercentOutput();

	public abstract double getCurrentOutput();

	public abstract void configCurrentLimit(int limit);

	public abstract double getFreeSpeedRPS();

	public abstract void setMeasurementPeriod(int periodMS);

	public abstract void flashMotor();

	public abstract void simulationConfig(PhysicsSim sim);
}
