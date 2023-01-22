package frc.team2412.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private final DrivebaseSubsystem drivebaseSubsystem;
	private final DoubleSupplier forward;
	private final DoubleSupplier strafe;
	private final DoubleSupplier rotation;

	// shuffleboard
	private GenericEntry driveSpeedEntry = Shuffleboard.getTab("Drivebase")
			.addPersistent("Drive Speed", 1)
			.withSize(2, 1)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("Min", 0, "Max", 5))
			.getEntry();
	private GenericEntry rotationSpeedEntry = Shuffleboard.getTab("Drivebase")
			.addPersistent("Rotation Speed", 1)
			.withSize(2, 1)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("Min", 0, "Max", 5))
			.getEntry();
	private GenericEntry fieldOrientedEntry = Shuffleboard.getTab("Drivebase")
			.addPersistent("Field Oriented", true)
			.withWidget(BuiltInWidgets.kToggleSwitch)
			.getEntry();

	public DriveCommand(
			DrivebaseSubsystem drivebaseSubsystem,
			DoubleSupplier forward,
			DoubleSupplier strafe,
			DoubleSupplier rotation) {
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.forward = forward;
		this.strafe = strafe;
		this.rotation = rotation;

		addRequirements(drivebaseSubsystem);
	}

	@Override
	public void execute() {
		double x = deadbandCorrection(-forward.getAsDouble());
		double y = deadbandCorrection(strafe.getAsDouble());
		double rot = deadbandCorrection(-rotation.getAsDouble());
		drivebaseSubsystem.drive(
				x * driveSpeedEntry.getDouble(1.0),
				y * driveSpeedEntry.getDouble(1.0),
				Rotation2d.fromDegrees(rot * rotationSpeedEntry.getDouble(1.0)),
				fieldOrientedEntry.getBoolean(true));
	}

	public double deadbandCorrection(double input) {
		return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
	}
}
