package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.*;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.*;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.arm.ManualArmOverrideCommand;
import frc.team2412.robot.commands.arm.ResetArmCommand;
import frc.team2412.robot.commands.arm.SetFullArmCommand;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.drivebase.DriveCommand;
import frc.team2412.robot.commands.intake.IntakeDefaultCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetStopCommand;
import frc.team2412.robot.commands.led.LEDPurpleCommand;
import frc.team2412.robot.commands.led.LEDYellowCommand;
import frc.team2412.robot.util.DriverAssist;

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

	// Arm

	public final Trigger armManualControl;

	public final Trigger armLowButton;
	public final Trigger armMiddleButton;
	public final Trigger armHighButton;
	public final Trigger armSubstationButton;
	public final Trigger armResetButton;

	public final Trigger wristRetractButton;
	// public final Trigger wristPrescoreButton;
	public final Trigger wristScoreButton;

	// intake
	public final Trigger intakeInButton;
	public final Trigger intakeOutButton;
	public final Trigger intakeStopButton;

	public final Trigger ledPurple;
	public final Trigger ledYellow;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		armManualControl = codriveController.rightStick();

		armLowButton = codriveController.y();
		armMiddleButton = codriveController.x();
		armHighButton = codriveController.a();
		armSubstationButton = codriveController.b();
		armResetButton = codriveController.start();

		wristRetractButton = driveController.povRight();
		wristScoreButton = driveController.povLeft();

		intakeInButton = driveController.a();
		intakeOutButton = driveController.y();
		intakeStopButton = driveController.b();

		ledPurple = driveController.rightBumper();
		ledYellow = driveController.leftBumper();

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

		// TODO Resolve control conflict with purple LED...
		driveController
				.rightBumper()
				.onTrue(new InstantCommand(() -> DriverAssist.alignRobot(s.drivebaseSubsystem)));

		driveController
				.rightBumper()
				.onFalse(new InstantCommand(() -> s.drivebaseSubsystem.getCurrentCommand().cancel()));
	}

	public void bindArmControls() {
		armManualControl.toggleOnTrue(
				new ManualArmOverrideCommand(
						s.armSubsystem, codriveController::getRightY, codriveController::getLeftY));
		armLowButton.onTrue(
				new SetFullArmCommand(s.armSubsystem, s.intakeSubsystem, ARM_LOW_POSITION, WRIST_RETRACT));
		armMiddleButton.onTrue(
				new SetFullArmCommand(
						s.armSubsystem, s.intakeSubsystem, ARM_MIDDLE_POSITION, WRIST_PRESCORE));
		armHighButton.onTrue(
				new SetFullArmCommand(
						s.armSubsystem, s.intakeSubsystem, ARM_HIGH_POSITION, WRIST_PRESCORE));
		armSubstationButton.onTrue(
				new SetFullArmCommand(
						s.armSubsystem, s.intakeSubsystem, ARM_SUBSTATION_POSITION, WRIST_PRESCORE));
		armResetButton.whileTrue(new ResetArmCommand(s.armSubsystem));

		wristRetractButton.onTrue(
				new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_RETRACT));
		// wristPrescoreButton.onTrue(new SetWristCommand(s.armSubsystem, s.intakeSubsystem,
		// WRIST_PRESCORE));
		wristScoreButton.onTrue(new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_SCORE));
	}

	public void bindIntakeControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(s.intakeSubsystem, new IntakeDefaultCommand(s.intakeSubsystem));

		intakeInButton.onTrue(
				new IntakeSetInCommand(s.intakeSubsystem).until(s.intakeSubsystem::isSecured));
		intakeOutButton.onTrue(new IntakeOutCommand(s.intakeSubsystem, s.ledSubsystem));
		intakeStopButton.onTrue(new IntakeSetStopCommand(s.intakeSubsystem));
	}

	public void bindLEDControls() {
		ledPurple.onTrue(new LEDPurpleCommand(s.ledSubsystem));
		ledYellow.onTrue(new LEDYellowCommand(s.ledSubsystem));
	}
}
