package frc.team2412.robot.subsystems;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.PFFController;
import frc.team2412.robot.util.gyroscope.Gyroscope;
import frc.team2412.robot.util.gyroscope.NavXGyro;
import frc.team2412.robot.util.gyroscope.Pigeon2Gyro;
import frc.team2412.robot.util.motorcontroller.BrushlessSparkMaxController;
import frc.team2412.robot.util.motorcontroller.MotorController;
import frc.team2412.robot.util.motorcontroller.MotorController.MotorControlMode;
import frc.team2412.robot.util.motorcontroller.MotorController.MotorNeutralMode;
import frc.team2412.robot.util.motorcontroller.TalonFXController;

public class DrivebaseSubsystem extends SubsystemBase {

	private final boolean IS_COMP = Robot.getInstance().isCompetition();

	// ordered from front left, front right, back left, back right
	private static final Rotation2d[] PRACTICE_DRIVEBASE_ENCODER_OFFSETS = {
		Rotation2d.fromDegrees(337.236),
		Rotation2d.fromDegrees(251.982),
		Rotation2d.fromDegrees(205.839),
		Rotation2d.fromDegrees(311.396)
	};
	private static final Rotation2d[] COMP_DRIVEBASE_ENCODER_OFFSETS = {
		Rotation2d.fromDegrees(-111.796),
		Rotation2d.fromDegrees(-343.388),
		Rotation2d.fromDegrees(-21.796),
		Rotation2d.fromDegrees(-332.841)
	};

	public final double MAX_DRIVE_SPEED_METERS = 4.4196;
	public final Rotation2d MAX_ROTATION_SPEED = Rotation2d.fromRotations(2.2378);

	private final double TICKS_PER_ROTATION = IS_COMP ? 1.0 : 2048.0;
	private final double WHEEL_DIAMETER_METERS = 0.0889;
	private final double DRIVE_REDUCTION = IS_COMP ? 6.75 : 8.14;
	private final double STEER_REDUCTION = IS_COMP ? 150 / 7 : (32.0 / 15.0) * (60.0 / 10.0);

	private final double TIP_F = 0.01;
	private final double TIP_P = 0.05;
	private final double TIP_TOLERANCE = 5;

	// units: raw sensor units
	private final double STEER_POSITION_COEFFICIENT =
			(TICKS_PER_ROTATION / (2 * Math.PI)) * STEER_REDUCTION; // radians
	// per
	// tick
	private final double DRIVE_VELOCITY_COEFFICIENT =
			(TICKS_PER_ROTATION / (Math.PI * WHEEL_DIAMETER_METERS)) * DRIVE_REDUCTION; // ticks per meter

	// Balance controller is in degrees
	private final PFFController<Double> balanceController;

	private final MotorController[] moduleDriveMotors =
			IS_COMP
					? new BrushlessSparkMaxController[] {
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
					}
					: new TalonFXController[] {
						new TalonFXController(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
					};

	private final MotorController[] moduleAngleMotors =
			IS_COMP
					? new BrushlessSparkMaxController[] {
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
					}
					: new TalonFXController[] {
						new TalonFXController(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
					};

	private final WPI_CANCoder[] moduleEncoders = {
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_RIGHT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_RIGHT_ENCODER_PORT)
	};

	private final Rotation2d[] moduleOffsets =
			IS_COMP ? COMP_DRIVEBASE_ENCODER_OFFSETS : PRACTICE_DRIVEBASE_ENCODER_OFFSETS;

	// 2ft x 2ft for practice bot
	private final Translation2d[] moduleLocations =
			IS_COMP
					? new Translation2d[] {
						new Translation2d(
								Units.inchesToMeters(12.375), Units.inchesToMeters(10.375)), // front left
						new Translation2d(
								Units.inchesToMeters(12.375), Units.inchesToMeters(-10.375)), // front right
						new Translation2d(
								Units.inchesToMeters(-12.375), Units.inchesToMeters(10.375)), // back left
						new Translation2d(
								Units.inchesToMeters(-12.375), Units.inchesToMeters(-10.375)) // back right
					}
					: new Translation2d[] {
						new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(8.5)), // front left
						new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(-8.5)), // front right
						new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(8.5)), // back left
						new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(-8.5)) // back right
					};

	SwerveDriveKinematics kinematics =
			new SwerveDriveKinematics(
					moduleLocations[0], moduleLocations[1], moduleLocations[2], moduleLocations[3]);

	private Gyroscope gyroscope;

	private SwerveDrivePoseEstimator poseEstimator;
	private Pose2d pose;

	private Field2d field = new Field2d();

	public DrivebaseSubsystem() {
		gyroscope = IS_COMP ? new Pigeon2Gyro(Hardware.GYRO_PORT) : new NavXGyro(SerialPort.Port.kMXP);

		odometry = new SwerveDriveOdometry(kinematics, gyroscope.getAngle(), getModulePositions());
		pose = odometry.getPoseMeters();
		poseEstimator =
				new SwerveDrivePoseEstimator(
						kinematics,
						Rotation2d.fromDegrees(gyroscope.getYaw()),
						getModulePositions(),
						new Pose2d());
		pose = poseEstimator.getEstimatedPosition();

		balanceController =
				PFFController.ofDouble(TIP_F, TIP_P)
						.setTargetPosition(gyroscope.getRawRoll().getDegrees())
						.setTargetPositionTolerance(TIP_TOLERANCE);

		// configure encoders offsets
		for (int i = 0; i < moduleEncoders.length; i++) {
			moduleEncoders[i].configFactoryDefault();
			moduleEncoders[i].configSensorInitializationStrategy(
					SensorInitializationStrategy.BootToAbsolutePosition);
		}

		// configure drive motors
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			MotorController driveMotor = moduleDriveMotors[i];
			driveMotor.setNeutralMode(MotorController.MotorNeutralMode.BRAKE);

			if (IS_COMP) {
				driveMotor.setControlMode(MotorControlMode.VOLTAGE);
			} else {
				driveMotor.setControlMode(MotorControlMode.VELOCITY);
				driveMotor.setPID(0.1, 0.001, 1023.0 / 20660.0);
			}
		}

		// configure angle motors
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			MotorController steeringMotor = moduleAngleMotors[i];
			steeringMotor.configFactoryDefault();
			steeringMotor.setNeutralMode(MotorNeutralMode.COAST);
			// Configure PID values
			if (IS_COMP) {
				steeringMotor.setPID(0.15, 0, 0);
			} else {
				steeringMotor.setPID(0.15, 0.00, 1.0);
			}

			steeringMotor.useIntegratedEncoder();

			if (IS_COMP) {
				steeringMotor.setInverted(true);
			}

			steeringMotor.setIntegratedEncoderPosition(
					getModuleAngles()[i].getRadians() * STEER_POSITION_COEFFICIENT);

			steeringMotor.setControlMode(MotorControlMode.POSITION);
		}

		// configure shuffleboard
		SmartDashboard.putData("Field", field);
	}

	/** Drives the robot using forward, strafe, and rotation. Units in meters */
	public void drive(
			double forward,
			double strafe,
			Rotation2d rotation,
			boolean fieldOriented,
			boolean autoBalance) {
		// Auto balancing will only be used in autonomous
		if (autoBalance) {
			forward -= balanceController.update(gyroscope.getRawRoll().getDegrees());
		}

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

		if (fieldOriented) {
			chassisSpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							forward, -strafe, rotation.getRadians(), gyroscope.getAngle());
		} else {
			chassisSpeeds = new ChassisSpeeds(forward, -strafe, rotation.getRadians());
		}
		drive(chassisSpeeds);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] moduleStates = getModuleStates(chassisSpeeds);
		if (Math.abs(chassisSpeeds.vxMetersPerSecond) <= 0.01
				&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= 0.01
				&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.01) {
			moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
			moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
			moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
			moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
		}
		drive(moduleStates);
	}

	/** Drives the robot using states */
	public void drive(SwerveModuleState[] states) {
		for (int i = 0; i < states.length; i++) {
			// states[i] = ModuleUtil.optimize(states[i], getModuleAngles()[i]);
			states[i] = SwerveModuleState.optimize(states[i], getModuleAngles()[i]);
		}

		// Set motor speeds and angles
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			if (IS_COMP) {
				moduleDriveMotors[i].set(
						states[i].speedMetersPerSecond / MAX_DRIVE_SPEED_METERS * 12); // set voltage
			} else {
				moduleDriveMotors[i].set(
						states[i].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT); // set velocity
			}
		}
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			moduleAngleMotors[i].set(states[i].angle.getRadians() * STEER_POSITION_COEFFICIENT);
		}
	}

	/**
	 * Array with modules with front left at [0], front right at [1], back left at [2], back right at
	 * [3]
	 */
	public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
		return kinematics.toSwerveModuleStates(speeds);
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (int i = 0; i < moduleDriveMotors.length; i++) {
			positions[i] =
					new SwerveModulePosition(
							moduleDriveMotors[i].getIntegratedEncoderPosition()
									* (1 / DRIVE_VELOCITY_COEFFICIENT),
							Rotation2d.fromRadians(
									moduleAngleMotors[i].getIntegratedEncoderPosition()
											* (1 / STEER_POSITION_COEFFICIENT)));
		}

		return positions;
	}

	/** Returns the module angles using encoders */
	public Rotation2d[] getModuleAngles() {
		Rotation2d[] rotations = new Rotation2d[4];
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			rotations[i] =
					Rotation2d.fromDegrees(
							(moduleEncoders[i].getAbsolutePosition() - moduleOffsets[i].getDegrees()));
		}
		return rotations;
	}

	/** Returns the kinematics */
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	/**
	 * Resets the gyroscope's angle to 0 After this is called, the radio (on bonk) or the intake (on
	 * comp) will be the robot's new global forward
	 */
	public void resetGyroAngle() {
		resetGyroAngle(gyroscope.getRawYaw());
	}

	/**
	 * Resets the robot's forward to the new angle relative to the radio (on bonk)
	 *
	 * @param angle The new forward
	 */
	public void resetGyroAngle(Rotation2d angle) {
		gyroscope.setAngleAdjustment(angle.unaryMinus());
	}

	/** Returns the robot's pose */
	public Pose2d getPose() {
		return pose;
	}

	/**
	 * Set's the robot's pose to the provided pose
	 *
	 * @param pose the new pose
	 */
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
		this.pose = pose;
	}

	/**
	 * Adds a vision measurement of the robot's pose.
	 *
	 * @param robotPoseMeters The robot pose in meters.
	 * @param timestampSeconds Timestamp in seconds since FPGA startup.
	 */
	public void addVisionMeasurement(Pose2d robotPoseMeters, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(robotPoseMeters, timestampSeconds);
	}

	/**
	 * Reset's the robot's pose to (0, 0) with rotation of 0. <br>
	 * Also resets the gyroscope
	 */
	public void resetPose() {
		resetGyroAngle();
		resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
	}

	public void simInit(PhysicsSim sim) {
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			// TODO: make work
			// sim.addTalonFX(moduleDriveMotors[i], 2, 20000, true);
			// sim.addTalonFX(moduleAngleMotors[i], 2, 20000);
		}
	}

	@Override
	public void periodic() {
		pose = poseEstimator.update(getGyroRotation2d(), getModulePositions());
		field.setRobotPose(pose);
	}
}
