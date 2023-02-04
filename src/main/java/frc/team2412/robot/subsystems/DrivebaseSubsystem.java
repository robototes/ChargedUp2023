package frc.team2412.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.ModuleUtil;
import frc.team2412.robot.util.PFFController;

public class DrivebaseSubsystem extends SubsystemBase {

	private static final double ticksPerRotation = 2048.0; // for the talonfx
	private static final double wheelDiameterMeters = 0.0889; // 3.5 inches
	private static final double driveReductionL1 = 8.14; // verified
	private static final double steerReduction = (32.0 / 15.0) * (60.0 / 10.0); // verified, 12.8

	// position units is one rotation / 2048
	// extrapolate this to meters using wheel perimeter (pi * wheel diameter)
	// raw sensor unit per meter driven = ticks/ perimeter

	// units: raw sensor units
	private static final double steerPositionCoefficient =
			(ticksPerRotation / (2 * Math.PI)) * steerReduction; // radians
	// per
	// tick
	private static final double driveVelocityCoefficient =
			(ticksPerRotation / (Math.PI * wheelDiameterMeters))
					* driveReductionL1; // ticks per meter per 100 ms

	private static final int talonFXLoopNumber = 0;
	private static final int canTimeoutMS = 20;

	private static PFFController<Double> balanceController;

	private static final double tipF = 0.01;
	private static final double tipP = 0.05;
	private static final double tipTolerance = 5;

	private WPI_TalonFX[] moduleDriveMotors = {
		new WPI_TalonFX(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
	};

	private WPI_TalonFX[] moduleAngleMotors = {
		new WPI_TalonFX(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
		new WPI_TalonFX(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
	};

	private WPI_CANCoder[] moduleEncoders = {
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_RIGHT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_RIGHT_ENCODER_PORT)
	};

	private Rotation2d[] moduleOffsets = {
		Hardware.DRIVEBASE_FRONT_LEFT_ENCODER_OFFSET,
		Hardware.DRIVEBASE_FRONT_RIGHT_ENCODER_OFFSET,
		Hardware.DRIVEBASE_BACK_LEFT_ENCODER_OFFSET,
		Hardware.DRIVEBASE_BACK_RIGHT_ENCODER_OFFSET
	};

	// 2ft x 2ft for practice bot
	private final Translation2d[] moduleLocations = {
		new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(8.5)), // front left
		new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(-8.5)), // front right
		new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(8.5)), // back left
		new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(-8.5)) // back right
	};

	SwerveDriveKinematics kinematics =
			new SwerveDriveKinematics(
					moduleLocations[0], moduleLocations[1], moduleLocations[2], moduleLocations[3]);

	private AHRS gyroscope;

	private SwerveDrivePoseEstimator poseEstimator;
	private Pose2d pose;

	private Field2d field = new Field2d();

	public DrivebaseSubsystem() {
		gyroscope = new AHRS(SerialPort.Port.kMXP);

		poseEstimator =
				new SwerveDrivePoseEstimator(
						kinematics,
						Rotation2d.fromDegrees(gyroscope.getYaw()),
						getModulePositions(),
						new Pose2d());
		pose = poseEstimator.getEstimatedPosition();

		balanceController =
				PFFController.ofDouble(tipF, tipP)
						.setTargetPosition((double) gyroscope.getRoll())
						.setTargetPositionTolerance(tipTolerance);

		// configure encoders offsets
		for (int i = 0; i < moduleEncoders.length; i++) {
			moduleEncoders[i].configFactoryDefault();
		}

		// configure drive motors
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			WPI_TalonFX driveMotor = moduleDriveMotors[i];
			driveMotor.setNeutralMode(NeutralMode.Brake);
			driveMotor.setSensorPhase(true);

			driveMotor.configNeutralDeadband(0.01);
			driveMotor.configSelectedFeedbackSensor(
					TalonFXFeedbackDevice.IntegratedSensor, talonFXLoopNumber, canTimeoutMS);

			driveMotor.config_kP(talonFXLoopNumber, 0.1);
			driveMotor.config_kI(talonFXLoopNumber, 0.001);
			driveMotor.config_kD(talonFXLoopNumber, 1023.0 / 20660.0);
		}

		// configure angle motors
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			WPI_TalonFX steeringMotor = moduleAngleMotors[i];
			steeringMotor.configFactoryDefault();
			steeringMotor.configSelectedFeedbackSensor(
					TalonFXFeedbackDevice.IntegratedSensor, talonFXLoopNumber, canTimeoutMS);
			// Configure PID values
			steeringMotor.config_kP(talonFXLoopNumber, 0.15, canTimeoutMS);
			steeringMotor.config_kI(talonFXLoopNumber, 0.00, canTimeoutMS);
			steeringMotor.config_kD(talonFXLoopNumber, 1.0, canTimeoutMS);
			steeringMotor.setSelectedSensorPosition(
					getModuleAngles()[i].times(((ticksPerRotation / 360) * steerReduction)).getDegrees());
		}

		// configure shuffleboard
		SmartDashboard.putData("Field", field);
	}

	/**
	 * Drives the robot using forward, strafe, and rotation. Units in meters
	 *
	 * @param forward
	 * @param strafe
	 * @param rotation
	 * @param fieldOriented
	 */
	public void drive(
			double forward,
			double strafe,
			Rotation2d rotation,
			boolean fieldOriented,
			boolean autoBalance) {
		// Auto balancing will only be used in autonomous
		if (autoBalance) {
			forward -= balanceController.update((double) gyroscope.getRoll());
		}

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

		if (fieldOriented) {
			chassisSpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							forward, -strafe, rotation.getRadians(), getGyroRotation2d());
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

	/**
	 * Drives the robot using states
	 *
	 * @param states
	 */
	public void drive(SwerveModuleState[] states) {
		for (int i = 0; i < states.length; i++) {
			states[i] = ModuleUtil.optimize(states[i], getModuleAngles()[i]);
		}

		// Set motor speeds and angles
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			// meters/100ms * raw sensor units conversion
			moduleDriveMotors[i].set(
					TalonFXControlMode.Velocity,
					((states[i].speedMetersPerSecond) / 10) * driveVelocityCoefficient);
		}
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			moduleAngleMotors[i].set(
					TalonFXControlMode.Position, states[i].angle.getRadians() * steerPositionCoefficient);
		}
	}

	/**
	 * @param speeds
	 * @return Array with modules with front left at [0], front right at [1], back left at [2], back
	 *     right at [3]
	 */
	public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
		return kinematics.toSwerveModuleStates(speeds);
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (int i = 0; i < moduleDriveMotors.length; i++) {
			positions[i] =
					new SwerveModulePosition(
							moduleDriveMotors[i].getSelectedSensorPosition() * (1 / driveVelocityCoefficient),
							Rotation2d.fromRadians(
									moduleAngleMotors[i].getSelectedSensorPosition()
											* (1 / steerPositionCoefficient)));
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
	 * Resets the gyroscope's angle to 0 After this is called, the radio (on bonk) will be the robot's
	 * new global forward
	 */
	public void resetGyroAngle() {
		resetGyroAngle(Rotation2d.fromDegrees(gyroscope.getYaw()));
	}

	/**
	 * Resets the robot's forward to the new angle relative to the radio (on bonk)
	 *
	 * @param angle The new forward
	 */
	public void resetGyroAngle(Rotation2d angle) {
		gyroscope.setAngleAdjustment(-angle.getDegrees());
	}

	/** Returns a Rotation2d containing the robot's rotation */
	public Rotation2d getGyroRotation2d() {
		return Rotation2d.fromDegrees(-gyroscope.getAngle());
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
			sim.addTalonFX(moduleDriveMotors[i], 2, 20000, true);
			sim.addTalonFX(moduleAngleMotors[i], 2, 20000);
		}
	}

	@Override
	public void periodic() {
		pose = poseEstimator.update(getGyroRotation2d(), getModulePositions());
		field.setRobotPose(pose);
		// Translation: X, Y, Z
		// Rotation: Pitch (up/down), yaw (left/right), roll (twist)
	}
}
