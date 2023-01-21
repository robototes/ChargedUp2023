package frc.team2412.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class DriveCommand extends CommandBase {
    private final DrivebaseSubsystem drivebaseSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    public DriveCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier forward, DoubleSupplier strafe,
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
        double rot = deadbandCorrection(rotation.getAsDouble()) / 2;
        drivebaseSubsystem.drive(x, y, Rotation2d.fromDegrees(rot), true);
    }

    public double deadbandCorrection(double input) {
        return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
    }
}
