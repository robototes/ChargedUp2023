package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.*;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_RETRACT;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_SCORE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_HIGH_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_LOW_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_MIDDLE_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_SUBSTATION_POSITION;

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

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;

		public static final double FAST_WRIST_EXTEND_TOLERANCE = 0.5;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

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

	public final Trigger ledPurple;
	public final Trigger ledYellow;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		armManualControlOn = codriveController.start();
		armManualControlOff = codriveController.back();

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

		ledPurple = codriveController.rightBumper();
		ledYellow = codriveController.leftBumper();

		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}
		if (Subsystems.SubsystemConstants.INTAKE_ENABLED) {
			bindIntakeControls();
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

	public void bindLEDControls() {
		ledPurple.onTrue(new LEDPurpleCommand(s.ledSubsystem));
		ledYellow.onTrue(new LEDYellowCommand(s.ledSubsystem));
	}
}
