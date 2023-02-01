package frc.team2412.robot.util.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public abstract class MotorController {
	public enum MotorNeutralMode {
		COAST {
			@Override
			public NeutralMode getCTRE() {
				return NeutralMode.Coast;
			}

			@Override
			public IdleMode getREV() {
				return IdleMode.kCoast;
			}
		},
		BRAKE {
			@Override
			public NeutralMode getCTRE() {
				return NeutralMode.Brake;
			}

			@Override
			public IdleMode getREV() {
				return IdleMode.kBrake;
			}
		};

		public NeutralMode getCTRE() {
			return NeutralMode.Coast;
		}

		public IdleMode getREV() {
			return IdleMode.kCoast;
		}
	}

	public enum MotorControlMode {
		POSITION {
			@Override
			public ControlMode getCTRE() {
				return ControlMode.Position;
			}

			@Override
			public ControlType getREV() {
				return ControlType.kPosition;
			}
		},
		VELOCITY {
			@Override
			public ControlMode getCTRE() {
				return ControlMode.Velocity;
			}

			@Override
			public ControlType getREV() {
				return ControlType.kVelocity;
			}
		},
		PERCENT {
			@Override
			public ControlMode getCTRE() {
				return ControlMode.PercentOutput;
			}

			@Override
			public ControlType getREV() {
				return ControlType.kDutyCycle;
			}
		};

		public ControlMode getCTRE() {
			return ControlMode.PercentOutput;
		}

		public ControlType getREV() {
			return ControlType.kDutyCycle;
		}
	}

	public abstract void setNeutralMode(MotorNeutralMode mode);

	public abstract void configFactoryDefault();

	public abstract void setEncoderInverted(boolean inverted);

	public abstract void setPID(double P, double I, double D);

	public abstract void set(double setpoint);

	public abstract void set(double setpoint, MotorControlMode mode);

	public abstract void setIntegratedEncoderPosition(double position);

	public abstract double getIntegratedEncoderPosition();

	public abstract void setControlMode(MotorControlMode mode);

	public abstract void useIntegratedEncoder();
}
