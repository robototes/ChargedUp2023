package frc.team2412.robot.util.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class Gyroscope {
	private Rotation2d adjustmentAngle = Rotation2d.fromRotations(0);
	private boolean inverted = false;
	private boolean simulated = false;
	private Rotation2d simulatedAngle = Rotation2d.fromRotations(0);

	public abstract Rotation2d getRawYaw();

	public abstract Rotation2d getRawPitch();

	public abstract Rotation2d getRawRoll();

	public void setInverted(boolean invert) {
		this.inverted = invert;
	}

	public boolean isInverted() {
		return inverted;
	}

	public Rotation2d getAngle() {
		if (simulated) {
			return simulatedAngle.plus(adjustmentAngle);
		}

		return inverted
				? getRawYaw().plus(adjustmentAngle).unaryMinus()
				: getRawYaw().plus(adjustmentAngle);
	}

	public void setAngleAdjustment(Rotation2d angleAdjustment) {
		adjustmentAngle = angleAdjustment;
	}

	public Rotation2d getAngleAdjustment() {
		return adjustmentAngle;
	}

	public void setSimulated(boolean simulated) {
		this.simulated = simulated;
	}

	/**
	 * Takes in an angle and adds it to the current angle
	 **/
	public void updateSimulatedAngle(Rotation2d angle) {
		this.simulatedAngle = this.simulatedAngle.plus(angle);
	}
}
