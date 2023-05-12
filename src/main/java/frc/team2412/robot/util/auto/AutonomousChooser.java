package frc.team2412.robot.util.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.function.Supplier;

public class AutonomousChooser {
	private static class AutoOption {
		private final String name;
		private final Supplier<Command> commandSupplier;
		private final List<PathPlannerTrajectory> pathGroup;

		public AutoOption(String optionName, String pathGroupName) {
			this(
					optionName,
					() -> AutonomousTrajectories.getAutoPathByName(pathGroupName),
					AutonomousTrajectories.getPathGroupByName(pathGroupName));
		}

		public AutoOption(
				String name, Supplier<Command> commandSupplier, List<PathPlannerTrajectory> pathGroup) {
			this.name = name;
			this.commandSupplier = commandSupplier;
			this.pathGroup = List.copyOf(pathGroup);
		}

		@Override
		public String toString() {
			return "AutoOption(name = "
					+ name
					+ ", commandSupplier = "
					+ commandSupplier
					+ ", pathGroup = "
					+ pathGroup
					+ ")";
		}

		public String getName() {
			return name;
		}

		public Command getCommand() {
			return commandSupplier.get();
		}

		public List<PathPlannerTrajectory> getPathGroup() {
			return pathGroup;
		}
	}

	private final SendableChooser<AutoOption> autonomousModeChooser = new SendableChooser<>();
	private final AutonomousField autonomousField = new AutonomousField(2);

	// auto naming scheme (Kind of jank but would be weird to change it up now so let's keep it
	// consistent)
	// STARTING POSITION
	//		Flat, Charge, Bump
	// followed by ACTIONS (multiple possible)
	//		LeaveCom, Charge, ScoreHigh, ScoreLow, Shoot, Pickup

	public AutonomousChooser() {
		setDefaultOption("FlatLeaveCom");
		addOption("BumpLeaveCom");
		addOption("ChargeLeaveComCharge");

		addOption("BumpScoreHighLeaveCom");
		addOption("FlatScoreHighLeaveCom");
		// autonomousModeChooser.addOption(
		// 		"FlatScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("FlatPieceScore"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("BumpPieceScore"));
		// autonomousModeChooser.addOption( // maybe
		// 		"FlatScoreLeaveCom2", () ->
		// AutonomousTrajectories.getAutoPathByName("FlatScoreLeaveCom2"));
		// autonomousModeChooser.addOption(
		// 		"ChargePreloadSecPiece",
		// 		() -> AutonomousTrajectories.getAutoPathByName("ChargePreloadSecPiece"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreTwo", () -> AutonomousTrajectories.getAutoPathByName("BumpScoreTwo"));
		addOption("ChargeScoreHighLeaveComCharge");

		addOption("ChargeScoreHighCharge");

		addOption("FlatScoreHighLeaveComScoreHigh");

		addOption("ChargeShootChargeShoot");

		addOption("ChargeCharge");

		addOption("DONOTUSEBumpScoreHighLeaveComScoreHigh", "UTBumpScoreHighLeaveComScoreHigh");
		// autonomousModeChooser.addOption(
		// 		"ChargePickupBot", () -> AutonomousTrajectories.getAutoPathByName("ChargePickupBot"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreTwo", () -> AutonomousTrajectories.getAutoPathByName("BumpScoreTwo"));
		// autonomousModeChooser.addOption(
		// 		"FlatScoreThree", () -> AutonomousTrajectories.getAutoPathByName("FlatScoreThree"));

		// There are more paths that have been created in the deploy/pathplanner path, they should work
		// however they have not been tested and thus are not added here yet.
		ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

		autonomousTab.add("Choose Auto Mode", autonomousModeChooser).withPosition(0, 0).withSize(2, 1);
		autonomousTab
				.add("Selected Auto Path", autonomousField.getField())
				.withPosition(2, 0)
				.withSize(3, 2);
	}

	private void setDefaultOption(String name) {
		setDefaultOption(name, name);
	}

	private void setDefaultOption(String optionName, String pathGroupName) {
		autonomousModeChooser.setDefaultOption(optionName, new AutoOption(optionName, pathGroupName));
	}

	private void addOption(String name) {
		addOption(name, name);
	}

	private void addOption(String optionName, String pathGroupName) {
		autonomousModeChooser.addOption(optionName, new AutoOption(optionName, pathGroupName));
	}

	public Command getAuto() {
		System.out.println("Selected auto:" + autonomousModeChooser.getSelected());
		return autonomousModeChooser.getSelected().getCommand();
	}

	public void updateField() {
		AutoOption selectedOption = autonomousModeChooser.getSelected();
		autonomousField.update(selectedOption.getName(), selectedOption.getPathGroup());
	}
}
