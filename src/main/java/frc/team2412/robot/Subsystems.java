package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.VisionSubsystem;

public class Subsystems {
	public static class SubsystemConstants {
		public static final boolean DRIVEBASE_ENABLED = true;
		public static final boolean ARM_ENABLED = false;
		public static final boolean INTAKE_ENABLED = false;
		public static final boolean VISION_ENABLED = false;
		public static final boolean DRIVER_VIS_ENABLED = true;
	}

	public DrivebaseSubsystem drivebaseSubsystem;
	public ArmSubsystem armSubsystem;
	public IntakeSubsystem intakeSubsystem;
	public VisionSubsystem visionSubsystem;

	public SwerveDrivePoseEstimator poseEstimator;

	public Subsystems() {
		SwerveModulePosition[] pseudoPositions = new SwerveModulePosition[4];
		SwerveModulePosition defaultPosition = new SwerveModulePosition(0.0, new Rotation2d());
		for (int pseudoPosition = 0; pseudoPosition < pseudoPositions.length; pseudoPosition++) {
			pseudoPositions[pseudoPosition] = defaultPosition;
		}

		poseEstimator =
				new SwerveDrivePoseEstimator(
						DrivebaseSubsystem.kinematics, new Rotation2d(), pseudoPositions, new Pose2d());

		boolean comp = Robot.getInstance().isCompetition();

		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem(poseEstimator);
		}
		if (VISION_ENABLED) {
			visionSubsystem = new VisionSubsystem(poseEstimator);
		}
		if (DRIVER_VIS_ENABLED) {
			if (Hardware.DRIVER_VISION_PATH == null) {
				DriverStation.reportWarning("No driver vision camera connected!", false);
			} else {
				UsbCamera driverVisionCamera =
						CameraServer.startAutomaticCapture("Driver vision", Hardware.DRIVER_VISION_PATH);
				driverVisionCamera.setResolution(160, 120);
			}
		}
		if (!comp) {
			return;
		}
		if (ARM_ENABLED) {
			armSubsystem = new ArmSubsystem();
		}
		if (INTAKE_ENABLED) {
			intakeSubsystem = new IntakeSubsystem();
		}
	}
}
