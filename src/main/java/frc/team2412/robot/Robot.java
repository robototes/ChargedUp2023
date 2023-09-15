package frc.team2412.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.commands.intake.IntakeDefaultCommand;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.MACAddress;
import frc.team2412.robot.util.auto.AutonomousChooser;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Optional;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
	private static Robot instance = null;

	enum RobotType {
		COMPETITION,
		DRIVEBASE;
	}

	public static Robot getInstance() {
		if (instance == null) instance = new Robot();
		return instance;
	}

	private final PowerDistribution PDP;

	public Controls controls;
	public Subsystems subsystems;

	private final RobotType robotType;
	public final Field2d field = new Field2d();
	public AutonomousChooser autonomousChooser;

	protected Robot(RobotType type) {
		instance = this;
		PDP = new PowerDistribution(Hardware.PDP_ID, ModuleType.kRev);
		robotType = type;
	}

	public double getVoltage() {
		return PDP.getVoltage();
	}

	protected Robot() {
		this(getTypeFromAddress());
	}

	public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x33, 0x9d, 0xd1);
	public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x28, 0x40, 0x82);

	private static RobotType getTypeFromAddress() {
		if (PRACTICE_ADDRESS.exists()) return RobotType.DRIVEBASE;
		else return RobotType.COMPETITION;
	}

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();
		LiveWindow.enableTelemetry(PDP);

		subsystems = new Subsystems();
		controls = new Controls(subsystems);
		autonomousChooser = new AutonomousChooser();

		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.enableNoMotionCalibration();
		}

		Shuffleboard.startRecording();

		if (RobotBase.isReal()) {
			DataLogManager.start();
			DriverStation.startDataLog(DataLogManager.getLog(), true);
		}

		CommandScheduler.getInstance()
				.onCommandInitialize(
						command -> System.out.println("Command initialized: " + command.getName()));
		CommandScheduler.getInstance()
				.onCommandInterrupt(
						command -> System.out.println("Command interrupted: " + command.getName()));
		CommandScheduler.getInstance()
				.onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

		SmartDashboard.putData("Field", field);
		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(subsystems.drivebaseSubsystem);
		SmartDashboard.putData(subsystems.armSubsystem);
		SmartDashboard.putData(subsystems.ledSubsystem);
		SmartDashboard.putData(subsystems.intakeSubsystem);
		SmartDashboard.putData(subsystems.visionSubsystem);
		DriverStation.silenceJoystickConnectionWarning(true);

		PathPlannerServer.startServer(5811);

		logRobotInfo();
	}

	private void logRobotInfo() {
		try {
			File gitInfoFile = new File(Filesystem.getDeployDirectory(), "git-info.txt");
			System.out.println("Git info:\n" + Files.readString(gitInfoFile.toPath()));
		} catch (IOException e) {
			DriverStation.reportWarning("Could not open git info file", true);
		}
	}

	public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
		if (subsystems.drivebaseSubsystem != null) {
			return new SwerveAutoBuilder(
					subsystems.drivebaseSubsystem::getPose, // Pose2d supplier
					subsystems.drivebaseSubsystem
							::resetPose, // Pose2d consumer, used to reset odometry at the beginning of
					// auto
					subsystems.drivebaseSubsystem.getKinematics(), // SwerveDriveKinematics
					new PIDConstants(
							5.0, 0.0,
							0.0), // PID constants to correct for translation error (used to create the X and
					// Y
					// PID controllers)
					new PIDConstants(
							3.0, 0.0,
							0.0), // PID constants to correct for rotation error (used to create the rotation
					// controller)
					subsystems.drivebaseSubsystem
							::drive, // Module states consumer used to output to the drive subsystem
					eventMap,
					true, // Should the path be automatically mirrored depending on alliance color.
					// Optional, defaults to true
					subsystems
							.drivebaseSubsystem // The drive subsystem. Used to properly set the requirements
					// of
					// path following commands
					);
		} else {
			return null;
		}
	}

	@Override
	public void testInit() {}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		Shuffleboard.startRecording();
		if (subsystems.armSubsystem != null) {
			// if manual arm control is enabled in auto, the arm will never move
			subsystems.armSubsystem.setManualOverride(false);
		}
		if (subsystems.intakeSubsystem != null) {
			new IntakeDefaultCommand(subsystems.intakeSubsystem).schedule();
		}
		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.setUseVisionMeasurements(false);
			// TODO: change this to not be hardcoded
			subsystems.drivebaseSubsystem.resetGyroAngleWithOrientation(Rotation2d.fromDegrees(180));
			autonomousChooser.getAuto().schedule();
			subsystems.drivebaseSubsystem.disableNoMotionCalibration();
		}
		if (subsystems.armLedSubsystem != null) {
			subsystems.armLedSubsystem.setLEDAutonomous();
		}
		if (subsystems.visionSubsystem != null) {
			subsystems.visionSubsystem.setAlliance(DriverStation.getAlliance());
		}
		// Checks if FMS is attatched and enables joystick warning if true
		DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());
	}

	@Override
	public void teleopInit() {
		Shuffleboard.startRecording();

		if (subsystems.armLedSubsystem != null) {
			subsystems.armLedSubsystem.setLEDAlliance();
		}
		if (subsystems.visionSubsystem != null) {
			subsystems.visionSubsystem.setAlliance(DriverStation.getAlliance());
		}
		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.setUseVisionMeasurements(true);
		}
	}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
		subsystems.drivebaseSubsystem.stopAllMotors();
	}

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.stopAllMotors();
		}
		if (subsystems.armSubsystem != null) {
			subsystems.armSubsystem.resetToLow();
		}
	}

	@Override
	public void simulationInit() {
		PhysicsSim sim = PhysicsSim.getInstance();
		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			subsystems.drivebaseSubsystem.simInit(sim);
		}
		if (Subsystems.SubsystemConstants.ARM_ENABLED) {
			subsystems.armSubsystem.simInit(sim);
		}
	}

	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public RobotType getRobotType() {
		return robotType;
	}

	public boolean isCompetition() {
		return getRobotType() == RobotType.COMPETITION;
	}

	private boolean wasArmButtonPressed = false;
	private boolean isArmCoast = false;

	private Optional<Boolean> wasAlignmentCorrect = Optional.empty();

	@Override
	public void disabledInit() {
		Shuffleboard.stopRecording();
		if (subsystems.armSubsystem != null) {
			wasArmButtonPressed = subsystems.armSubsystem.isIdleModeTogglePressed();
		}
		wasAlignmentCorrect = Optional.empty();
	}

	@Override
	public void disabledPeriodic() {
		if (subsystems.armSubsystem != null) {
			boolean isArmButtonPressed = subsystems.armSubsystem.isIdleModeTogglePressed();
			if (!wasArmButtonPressed && isArmButtonPressed) {
				if (isArmCoast) {
					subsystems.armSubsystem.setBrake();
				} else {
					subsystems.armSubsystem.setCoast();
				}
				isArmCoast = !isArmCoast;
			}
			wasArmButtonPressed = isArmButtonPressed;
		}

		if ((subsystems.visionSubsystem != null) && (subsystems.armLedSubsystem != null)) {
			boolean isAlignmentCorrect = subsystems.visionSubsystem.isYawAlignedToGrid();
			if (wasAlignmentCorrect.isEmpty() || (wasAlignmentCorrect.get() != isAlignmentCorrect)) {
				if (isAlignmentCorrect) {
					subsystems.armLedSubsystem.setLEDCorrectAlignment();
				} else {
					subsystems.armLedSubsystem.setLEDIncorrectAlignment();
				}
				wasAlignmentCorrect = Optional.of(isAlignmentCorrect);
			}
		}
	}

	@Override
	public void disabledExit() {
		if (subsystems.armSubsystem != null) {
			subsystems.armSubsystem.setBrake();
		}
		isArmCoast = false;
	}
}
