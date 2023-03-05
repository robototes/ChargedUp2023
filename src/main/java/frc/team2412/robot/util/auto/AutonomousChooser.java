package frc.team2412.robot.util.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousChooser {
	public interface CommandSupplier {
		Command getCommand();
	}

	private final SendableChooser<CommandSupplier> autonomousModeChooser = new SendableChooser<>();

	public AutonomousChooser() {
		autonomousModeChooser.setDefaultOption(
				"LeftLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("TopLeaveCom"));
		autonomousModeChooser.addOption(
				"RightLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("BotLeaveCom"));
		autonomousModeChooser.addOption(
				"MiddleLeaveComCharge",
				() -> AutonomousTrajectories.getAutoPathByName("MiddleLeaveComCharge"));
		autonomousModeChooser.addOption(
				"LeftScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("TopScoreLeaveCom"));
		autonomousModeChooser.addOption(
				"MiddleScoreLeaveComCharge",
				() -> AutonomousTrajectories.getAutoPathByName("MiddleScoreLeaveComCharge"));
		autonomousModeChooser.addOption(
				"RightScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("BotScoreLeaveCom"));
		autonomousModeChooser.addOption(
				"LeftScoreLeaveCom2", () -> AutonomousTrajectories.getAutoPathByName("TopScoreLeaveCom2"));

		// There are more paths that have been created in the deploy/pathplanner path, they should work
		// however they have not been tested and thus are not added here yet.
		ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

		autonomousTab.add("Choose Auto Mode", autonomousModeChooser).withPosition(0, 0).withSize(2, 1);
	}

	public Command getAuto() {
		return autonomousModeChooser.getSelected().getCommand();
	}
}
