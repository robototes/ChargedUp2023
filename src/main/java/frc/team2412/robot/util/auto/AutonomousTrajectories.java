package frc.team2412.robot.util.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	public static Command getChargedAutoPathCommand() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("MiddleLeaveCom&Charge", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getCommunityAutoPathCommand() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("BottomLeaveCommunity", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
}
