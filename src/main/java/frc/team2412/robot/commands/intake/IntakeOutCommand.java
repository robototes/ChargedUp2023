package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.led.LEDOffCommand;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LEDSubsystem;

public class IntakeOutCommand extends SequentialCommandGroup {

	public IntakeOutCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
		addCommands(new IntakeSetOutCommand(intakeSubsystem), new LEDOffCommand(ledSubsystem));
	}
}
