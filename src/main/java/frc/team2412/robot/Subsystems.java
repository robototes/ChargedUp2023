package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
		public static final boolean VISION_ENABLED = true;
		public static final boolean DRIVER_VIS_ENABLED = true;
	}

	public DrivebaseSubsystem drivebaseSubsystem;
	public ArmSubsystem armSubsystem;
	public IntakeSubsystem intakeSubsystem;
	public VisionSubsystem visionSubsystem;

	public Subsystems() {
		boolean comp = Robot.getInstance().isCompetition();

		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem();
			if (VISION_ENABLED) {
				visionSubsystem = new VisionSubsystem(drivebaseSubsystem::addVisionMeasurement);
			}
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
