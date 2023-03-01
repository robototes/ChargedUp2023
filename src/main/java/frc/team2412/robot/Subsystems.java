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
import frc.team2412.robot.subsystems.LEDSubsystem;
import frc.team2412.robot.subsystems.VisionSubsystem;

public class Subsystems {
	public static class SubsystemConstants {
		public static final boolean DRIVEBASE_ENABLED = true;
		public static final boolean ARM_ENABLED = true;
		public static final boolean INTAKE_ENABLED = true;
		public static final boolean VISION_ENABLED = false;
		public static final boolean LED_ENABLED = true;
		public static final boolean DRIVER_VIS_ENABLED = false;
	}

	public DrivebaseSubsystem drivebaseSubsystem;
	public ArmSubsystem armSubsystem;
	public IntakeSubsystem intakeSubsystem;
	public VisionSubsystem visionSubsystem;
	public LEDSubsystem ledSubsystem;

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
				System.out.println("Starting automatic capture");
				UsbCamera driverVisionCamera =
						CameraServer.startAutomaticCapture("Driver vision", Hardware.DRIVER_VISION_PATH);
				var camInfo = driverVisionCamera.getInfo();
				System.out.println(
						"Connecting to "
								+ camInfo.name
								+ " device number "
								+ camInfo.dev
								+ " on path "
								+ camInfo.path);
				// Available resolutions:
				// 640x480, 160x120, 176x144, 320x180, 320x240, 352x288, 424x240, 480x270, 640x360, 800x448
				// < 30 YUYV FPS:
				// 800x600, 848x480, 960x540, 1024x576, 1280x720, 1600x896, 1920x1080
				// YUYV only:
				// 2304x1296, 2304x1536
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
		if (LED_ENABLED) {
			ledSubsystem = new LEDSubsystem();
		}
	}
}
