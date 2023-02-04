package frc.team2412.robot.util.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

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
		},
		VOLTAGE {
			@Override
			public ControlMode getCTRE() {
				return ControlMode.Velocity;
			}

			@Override
			public ControlType getREV() {
				return ControlType.kVelocity;
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

	public abstract void setInverted(boolean inverted);

	public abstract void setNominalVoltage(double voltage);
}
