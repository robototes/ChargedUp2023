package frc.team2412.robot.util.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class Gyroscope {
	private Rotation2d adjustmentAngle = Rotation2d.fromRotations(0);
	private boolean inverted = false;

	public abstract Rotation2d getRawYaw();

	public abstract Rotation2d getRawPitch();

	public abstract Rotation2d getRawRoll();

	public Rotation2d getYaw() {
		return getRawYaw().plus(adjustmentAngle);
	}

	public void setInverted(boolean invert) {
		this.inverted = invert;
	}

	public boolean isInverted() {
		return inverted;
	}

	public Rotation2d getAngle() {
		return inverted ? getRawYaw().minus(adjustmentAngle) : getRawYaw().plus(adjustmentAngle);
	}

	public void setAngleAdjustment(Rotation2d angleAdjustment) {
		adjustmentAngle = angleAdjustment;
	}

	public Rotation2d getAngleAdjustment() {
		return adjustmentAngle;
	}
}
