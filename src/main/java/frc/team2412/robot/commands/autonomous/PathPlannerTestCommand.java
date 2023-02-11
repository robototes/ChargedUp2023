package frc.team2412.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class PathPlannerTestCommand {
	//these define where the robot is starting on the field
	public static enum startPoint {
		TOP, MIDDLE, BOTTOM;
	}
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

	public static Command createAutoPath(startPoint startingPosition, boolean score, boolean chargeStation) {
		List<PathPoint> pathPoints = new ArrayList<PathPoint>();
		switch (startingPosition) {
			case TOP: //case TOP means that the robot would be starting at the top of the field relative to the pathPlanner interface
				if (score) { //score in top goal
					pathPoints.add(createPathPoint(1.7, 4.7, 180, 180));
				}
				//leave community
				pathPoints.add(createPathPoint(4.8, 4.7, 0, 0));
				break;
			case MIDDLE:
				break;
			case BOTTOM:
				break;
		}
		//go to charge station
		if (chargeStation) {
			pathPoints.add(createPathPoint(3.9, 2.8, 180, 0));
		}
		//generate and return path with created points
		PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(1, 0.3), pathPoints);
		Command auto = Robot.autoBuilder.fullAuto(path);
		return auto;
	}

	//creates and returns a path point with inputed values
	private static PathPoint createPathPoint(double translationX, double translationY, double headingRotation, double holonomicRotation) {
		return new PathPoint(new Translation2d(translationX, translationY), Rotation2d.fromDegrees(headingRotation), Rotation2d.fromDegrees(holonomicRotation));
	}
}
