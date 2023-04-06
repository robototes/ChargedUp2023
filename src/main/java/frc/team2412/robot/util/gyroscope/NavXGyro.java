package frc.team2412.robot.util.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class NavXGyro extends Gyroscope {
	private final AHRS navx;

	public NavXGyro(Port serialPort) {
		this.navx = new AHRS(serialPort);
	}

	@Override
	public Rotation2d getRawYaw() {
		return Rotation2d.fromDegrees(this.navx.getYaw());
	}

	@Override
	public Rotation2d getRawPitch() {
		return Rotation2d.fromDegrees(this.navx.getPitch());
	}

	@Override
	public Rotation2d getRawRoll() {
		return Rotation2d.fromDegrees(this.navx.getRoll());
	}

	@Override
	public void startLogging() {
		// nothing for navx
	}
}
