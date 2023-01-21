package frc.team2412.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Hardware {
    public static final int PDP_ID = 1;

    // Drive devices are from range 1-19
    public static final int DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR = 1, DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR = 4,
            DRIVEBASE_BACK_LEFT_DRIVE_MOTOR = 7, DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR = 2, DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR = 5,
            DRIVEBASE_BACK_LEFT_ANGLE_MOTOR = 8, DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR = 11;
    public static final int DRIVEBASE_FRONT_LEFT_ENCODER_PORT = 3, DRIVEBASE_FRONT_RIGHT_ENCODER_PORT = 6,
            DRIVEBASE_BACK_LEFT_ENCODER_PORT = 9, DRIVEBASE_BACK_RIGHT_ENCODER_PORT = 12;
    public static final Rotation2d DRIVEBASE_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromDegrees(337.236);
    public static final Rotation2d DRIVEBASE_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromDegrees(251.982);
    public static final Rotation2d DRIVEBASE_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromDegrees(205.839);
    public static final Rotation2d DRIVEBASE_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromDegrees(311.396);
    public static final int GYRO_PORT = 13;

    public static final double GEAR_RATIO;

    static {
        GEAR_RATIO = Robot.getInstance().isCompetition()
                ? 0 // TBD
                : 8.14; // L1 drive ratio
    }
}
