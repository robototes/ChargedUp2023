package frc.team2412.robot.util.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	private static final Subsystems s = Robot.getInstance().subsystems;
	//	public static final SetWristCommand wristOut =
	//			new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_SCORE);
	//	public static final IntakeSetOutCommand intakeOut = new IntakeSetOutCommand(s.intakeSubsystem);
	//	public static final SetWristCommand wristIn =
	//			new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_RETRACT);
	//	public static final SequentialCommandGroup score =
	//			new SequentialCommandGroup(wristOut, intakeOut.withTimeout(1.5), wristIn);
	//	public static final Command intake =
	//			new IntakeSetInCommand(s.intakeSubsystem).until(s.intakeSubsystem::isSecured);

	public static Command getChargedAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("MiddleLeaveComCharge", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		// eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getTopCommunityAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("TopLeaveCom", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		s.drivebaseSubsystem.resetPose();
		// eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getBotCommunityAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("BotLeaveCom", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		// eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getBotScoreAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("BotPieceScore", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		// eventMap.put("score", score);
		// eventMap.put("intake", intake);
		// eventMap.put("wristIn", wristIn);
		// eventMap.put("wristOut", wristOut);
		// eventMap.put("intakeOut", intakeOut);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getTopScoreAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("topPieceScore", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		//		eventMap.put("score", score);
		//		eventMap.put("intake", intake);
		//		eventMap.put("wristIn", wristIn);
		//		eventMap.put("wristOut", wristOut);
		//		eventMap.put("intakeOut", intakeOut);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
}
