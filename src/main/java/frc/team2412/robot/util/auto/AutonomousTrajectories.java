package frc.team2412.robot.util.auto;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_RETRACT;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_SCORE;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	private static final Subsystems s = Robot.getInstance().subsystems;
	public static final SetWristCommand wristOut =
			new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_SCORE);
	public static final IntakeOutCommand intakeOut =
			new IntakeOutCommand(s.intakeSubsystem, s.ledSubsystem);
	public static final SetWristCommand wristIn =
			new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WRIST_RETRACT);
	public static final SequentialCommandGroup score =
			new SequentialCommandGroup(wristOut, intakeOut, wristIn);
	public static final Command intake =
			new IntakeSetInCommand(s.intakeSubsystem).until(s.intakeSubsystem::isSecured);

	public static Command getChargedAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("MiddleLeaveComCharge", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getTopCommunityAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("TopLeaveCom", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getBotCommunityAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("BotLeaveCom", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put("score", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getBotScoreAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("BotPieceScore", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put("score", score);
		eventMap.put("intake", intake);
		eventMap.put("wristIn", wristIn);
		eventMap.put("wristOut", wristOut);
		eventMap.put("intakeOut", intakeOut);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}

	public static Command getTopScoreAutoPath() {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup("topPieceScore", new PathConstraints(1, 0.3));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put("score", score);
		eventMap.put("intake", intake);
		eventMap.put("wristIn", wristIn);
		eventMap.put("wristOut", wristOut);
		eventMap.put("intakeOut", intakeOut);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
}
