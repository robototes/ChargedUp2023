package frc.team2412.robot.util.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousChooser {
	private final SendableChooser<Command> autonomousModeChooser = new SendableChooser<>();

	public AutonomousChooser() {
		autonomousModeChooser.addOption(
				"Community", AutonomousTrajectories.getCommunityAutoPathCommand());
		autonomousModeChooser.addOption(
				"ChargeStation", AutonomousTrajectories.getChargedAutoPathCommand());

		ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

		autonomousTab.add("Choose Auto Mode", autonomousModeChooser).withPosition(0, 0).withSize(2, 1);
	}

	public Command getAuto() {
		System.out.println(autonomousModeChooser.getSelected());
		return autonomousModeChooser.getSelected();
	}
}
