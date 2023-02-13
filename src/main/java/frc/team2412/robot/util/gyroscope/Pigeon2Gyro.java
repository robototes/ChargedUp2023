package frc.team2412.robot.util.gyroscope;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro extends Gyroscope {
	private final Pigeon2 pigeon;

	public Pigeon2Gyro(int id) {
		this.pigeon = new Pigeon2(id);
	}

	@Override
	public Rotation2d getRawYaw() {
		return Rotation2d.fromDegrees(this.pigeon.getYaw());
	}

	@Override
	public Rotation2d getRawPitch() {
		return Rotation2d.fromDegrees(this.pigeon.getPitch());
	}

	@Override
	public Rotation2d getRawRoll() {
		return Rotation2d.fromDegrees(this.pigeon.getRoll());
	}
}
