package frc.team2412.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;
import java.util.List;

public class PathPlannerTestCommand {
	/**
	 * Get a command that runs a drive path created in pathplanner
	 *
	 * @return the pathplanner auto command
	 */
	public static Command getAutoCommand() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("FullAuto", new PathConstraints(1, 0.3));
		Command fullAuto = Robot.autoBuilder.fullAuto(pathGroup);
		return fullAuto;
	}
}
