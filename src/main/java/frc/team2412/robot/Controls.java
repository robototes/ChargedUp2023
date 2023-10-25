package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.FAST_WRIST_EXTEND_TOLERANCE;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_RETRACT;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_SCORE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_HIGH_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_LOW_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_MIDDLE_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_SUBSTATION_POSITION;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.arm.ManualArmOverrideOffCommand;
import frc.team2412.robot.commands.arm.ManualArmOverrideOnCommand;
import frc.team2412.robot.commands.arm.SetFullArmCommand;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.drivebase.DriveCommand;
import frc.team2412.robot.commands.intake.IntakeDefaultCommand;
import frc.team2412.robot.commands.intake.IntakeSetFastOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import frc.team2412.robot.commands.led.LEDPurpleCommand;
import frc.team2412.robot.commands.led.LEDYellowCommand;
import frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType;
import frc.team2412.robot.util.DriverAssist;

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;

		public static final double FAST_WRIST_EXTEND_TOLERANCE = 0.5;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

	// Drivebase

	public final Trigger triggerDriverAssistCube;
	public final Trigger triggerDriverAssistCone;

	// Arm

	public final Trigger armManualControlOn;
	public final Trigger armManualControlOff;

	public final Trigger armLowButton;
	public final Trigger armMiddleButton;
	public final Trigger armHighButton;
	public final Trigger armSubstationButton;

	public final Trigger wristCarryButton;
	public final Trigger wristPrescoreButton;
	public final Trigger wristScoreButton;
	public final Trigger wristIntakeButton;

	// intake
	public final Trigger codriveIntakeInButton;
	public final Trigger codriveIntakeOutButton;
	// public final Trigger codriveIntakeStopButton;
	public final Trigger driveIntakeInButton;
	public final Trigger driveIntakeOutButton;
	public final Trigger driveIntakeFastOutButton;

	public final EventLoop bonkIntakeWristUpEvent;
	public final EventLoop bonkIntakeWristDownEvent;
	public final Trigger bonkIntakeWristUpTrigger;
	public final Trigger bonkIntakeWristDownTrigger;
	public final Trigger bonkIntakeInButton;
	public final Trigger bonkIntakeOutButton;
	public final Trigger bonkIntakeStopButton;

	public final Trigger ledPurple;
	public final Trigger ledYellow;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		armManualControlOn = codriveController.start();
		armManualControlOff = codriveController.back();
		triggerDriverAssistCube = driveController.leftBumper();
		triggerDriverAssistCone = driveController.rightBumper();

		armLowButton = codriveController.y();
		armMiddleButton = codriveController.x();
		armHighButton = codriveController.a();
		armSubstationButton = codriveController.b();

		wristCarryButton = codriveController.povDown();
		wristScoreButton = codriveController.povRight();
		wristPrescoreButton = codriveController.povLeft();
		wristIntakeButton = codriveController.povUp();

		codriveIntakeInButton = codriveController.rightTrigger();
		codriveIntakeOutButton = codriveController.leftTrigger();
		driveIntakeInButton = driveController.a();
		driveIntakeOutButton = driveController.y();
		driveIntakeFastOutButton = driveController.b();

		bonkIntakeWristUpEvent = new EventLoop();
		bonkIntakeWristDownEvent = new EventLoop();
		bonkIntakeWristUpTrigger = driveController.rightTrigger(0.1, bonkIntakeWristUpEvent);
		bonkIntakeWristDownTrigger = driveController.leftTrigger(bonkIntakeWristDownEvent, 0.1);
		bonkIntakeInButton = driveController.x();
		bonkIntakeOutButton = driveController.y();
		bonkIntakeStopButton = driveController.a();

		ledPurple = codriveController.rightBumper();
		ledYellow = codriveController.leftBumper();

		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}
		if (Subsystems.SubsystemConstants.INTAKE_ENABLED) {
			bindIntakeControls();
		}
		if (Subsystems.SubsystemConstants.BONK_INTAKE_ENABLED) {
			bindBonkIntakeControls();
		}
		if (Subsystems.SubsystemConstants.LED_ENABLED) {
			bindLEDControls();
		}
		if (Subsystems.SubsystemConstants.ARM_ENABLED) {
			bindArmControls();
		}
	}

	public void bindDrivebaseControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(
						s.drivebaseSubsystem,
						new DriveCommand(
								s.drivebaseSubsystem,
								driveController::getLeftY,
								driveController::getLeftX,
								driveController::getRightX,
								driveController::getRightTriggerAxis));
		driveController.start().onTrue(new InstantCommand(s.drivebaseSubsystem::resetGyroAngle));
		driveController.back().onTrue(new InstantCommand(s.drivebaseSubsystem::resetPose));
		driveController.leftStick().onTrue(new InstantCommand(s.drivebaseSubsystem::toggleXWheels));

		triggerDriverAssistCube.whileTrue(
				DriverAssist.alignRobotCommand(s.drivebaseSubsystem, GamePieceType.CUBE).repeatedly());
		triggerDriverAssistCone.whileTrue(
				DriverAssist.alignRobotCommand(s.drivebaseSubsystem, GamePieceType.CONE).repeatedly());
		// // CommandBase driverAssistCube =
		// // 		new ProxyCommand(() -> DriverAssist.alignRobot(s.drivebaseSubsystem,
		// GamePieceType.CUBE));
		// triggerDriverAssistCube.onTrue(
		// 		new ProxyCommand(() -> DriverAssist.alignRobot(s.drivebaseSubsystem, GamePieceType.CUBE)));
		// // CommandBase driverAssistCone =
		// // 		new ProxyCommand(() -> DriverAssist.alignRobot(s.drivebaseSubsystem,
		// GamePieceType.CONE));
		// triggerDriverAssistCone.onTrue(
		// 		new ProxyCommand(() -> DriverAssist.alignRobot(s.drivebaseSubsystem, GamePieceType.CONE)));

		// triggerDriverAssistCube.onFalse(
		// 		new InstantCommand(() -> s.drivebaseSubsystem.getCurrentCommand().cancel()));
		// triggerDriverAssistCone.onFalse(
		// 		new InstantCommand(() -> s.drivebaseSubsystem.getCurrentCommand().cancel()));
	}

	public void bindArmControls() {
		s.armSubsystem.setPresetAdjustJoysticks(
				codriveController::getRightY, codriveController::getLeftY);

		armManualControlOn.onTrue(
				new ManualArmOverrideOnCommand(
						s.armSubsystem, codriveController::getRightY, codriveController::getLeftY));
		armManualControlOff.onTrue(new ManualArmOverrideOffCommand(s.armSubsystem));
		armLowButton.onTrue(
				new SetFullArmCommand(
						s.armSubsystem, ARM_LOW_POSITION, WRIST_RETRACT, FAST_WRIST_EXTEND_TOLERANCE));
		armMiddleButton.onTrue(
				new SetFullArmCommand(s.armSubsystem, ARM_MIDDLE_POSITION, WRIST_PRESCORE));
		armHighButton.onTrue(new SetFullArmCommand(s.armSubsystem, ARM_HIGH_POSITION, WRIST_PRESCORE));
		armSubstationButton.onTrue(
				new SetFullArmCommand(
						s.armSubsystem, ARM_SUBSTATION_POSITION, WRIST_PRESCORE, FAST_WRIST_EXTEND_TOLERANCE));

		wristCarryButton.onTrue(new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_RETRACT));
		wristPrescoreButton.onTrue(new SetWristCommand(s.armSubsystem, WRIST_PRESCORE));
		wristScoreButton.onTrue(new SetWristCommand(s.armSubsystem, WRIST_SCORE));
		wristIntakeButton.onTrue(
				new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_PRESCORE));
	}

	public void bindIntakeControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(s.intakeSubsystem, new IntakeDefaultCommand(s.intakeSubsystem));

		// Drive Buttons

		driveIntakeInButton.onTrue(new IntakeSetInCommand(s.intakeSubsystem));
		driveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		driveIntakeFastOutButton.onTrue(new IntakeSetFastOutCommand(s.intakeSubsystem));

		// Codrive buttons
		codriveIntakeInButton.onTrue(new IntakeSetInCommand(s.intakeSubsystem));
		codriveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		// if (Subsystems.SubsystemConstants.LED_ENABLED) {
		// 	codriveIntakeOutButton.onTrue(new IntakeOutCommand(s.intakeSubsystem, s.ledSubsystem));
		// } else {
		// 	codriveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		// }
		// codriveIntakeStopButton.onTrue(new IntakeSetStopCommand(s.intakeSubsystem));
	}

	public void bindBonkIntakeControls() {
		bonkIntakeWristUpEvent.bind(() -> 
					s.bonkIntakeSubsystem.adjustWristCommand(driveController.getRightTriggerAxis()));
		// negative bc its going down
		bonkIntakeWristDownEvent.bind(() -> 
					s.bonkIntakeSubsystem.adjustWristCommand(-driveController.getLeftTriggerAxis()));

		bonkIntakeInButton.onTrue(s.bonkIntakeSubsystem.intakeInCommand());
		bonkIntakeOutButton.onTrue(s.bonkIntakeSubsystem.intakeOutCommand());
		bonkIntakeStopButton.onTrue(s.bonkIntakeSubsystem.intakeStopCommand());
	}

	public void bindLEDControls() {
		ledPurple.onTrue(new LEDPurpleCommand(s.ledSubsystem));
		ledYellow.onTrue(new LEDYellowCommand(s.ledSubsystem));
	}
}
