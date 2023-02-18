package frc.team2412.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private static final double TRIGGER_MODIFIER_DEFAULT = 0.7;

	private final DrivebaseSubsystem drivebaseSubsystem;
	private final DoubleSupplier forward;
	private final DoubleSupplier strafe;
	private final DoubleSupplier rotation;
	private final DoubleSupplier speedLimiter;

	// shuffleboard
	private static GenericEntry driveSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Drive Speed", 1)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();
	private static GenericEntry rotationSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Rotation Speed", 1)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();
	private static GenericEntry fieldOrientedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Field Oriented", true)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();
	private static GenericEntry cubeSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.add("Cube Speed", true)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();
	private static GenericEntry triggerModifierEntry =
			Shuffleboard.getTab("Drivebase")
					.add("Trigger Modifier", TRIGGER_MODIFIER_DEFAULT)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();

	public DriveCommand(
			DrivebaseSubsystem drivebaseSubsystem,
			DoubleSupplier forward,
			DoubleSupplier strafe,
			DoubleSupplier rotation,
			DoubleSupplier speedLimiter) {
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.forward = forward;
		this.strafe = strafe;
		this.rotation = rotation;
		// this variable give the right trigger input
		this.speedLimiter = speedLimiter;

		addRequirements(drivebaseSubsystem);
	}

	@Override
	public void execute() {
		// this is so ugly spotless
		double driveSpeedModifier =
				driveSpeedEntry.getDouble(1.0)
						* (1
								- (speedLimiter.getAsDouble()
										* (1 - triggerModifierEntry.getDouble(TRIGGER_MODIFIER_DEFAULT))));

		double x = deadbandCorrection(-forward.getAsDouble());
		double y = deadbandCorrection(strafe.getAsDouble());
		double rot = deadbandCorrection(-rotation.getAsDouble());

		// math for normalizing and cubing inputs
		double magnitude = Math.pow(Math.min(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 1), 3);
		double angle = Math.atan2(y, x);
		double cubed_x = magnitude * Math.cos(angle);
		double cubed_y = magnitude * Math.sin(angle);

		drivebaseSubsystem.drive(
				(cubeSpeedEntry.getBoolean(false) ? cubed_x : x)
						* driveSpeedModifier
						* DrivebaseSubsystem.MAX_DRIVE_SPEED_METERS_PER_SEC, // convert from percent to m/s
				(cubeSpeedEntry.getBoolean(false) ? cubed_y : y)
						* driveSpeedModifier
						* DrivebaseSubsystem.MAX_DRIVE_SPEED_METERS_PER_SEC,
				Rotation2d.fromRotations(
						rot
								* rotationSpeedEntry.getDouble(1.0)
								* DrivebaseSubsystem.MAX_ROTATIONS_PER_SEC
										.getRotations()), // convert from percent to rotations per second
				fieldOrientedEntry.getBoolean(true),
				false);
	}

	public double deadbandCorrection(double input) {
		return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
	}
}
