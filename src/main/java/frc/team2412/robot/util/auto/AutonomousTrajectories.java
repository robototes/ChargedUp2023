package frc.team2412.robot.util.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.arm.SetWristCommand.WristPosition;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	private static final Subsystems s = Robot.getInstance().subsystems;

	public static Command getAutoPathByName(String name) {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup(name, new PathConstraints(2, 2.0));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		SetWristCommand wristOut =
				new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WristPosition.WRIST_SCORE);
		SetWristCommand wristPrescore =
				new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WristPosition.WRIST_PRESCORE);
		IntakeSetOutCommand intakeOut = new IntakeSetOutCommand(s.intakeSubsystem);
		IntakeSetInCommand intakeIn = new IntakeSetInCommand(s.intakeSubsystem);
		SetWristCommand wristIn =
				new SetWristCommand(s.armSubsystem, s.intakeSubsystem, WristPosition.WRIST_RETRACT);
		SequentialCommandGroup score =
				new SequentialCommandGroup(intakeIn, wristOut, intakeOut.withTimeout(1.5), wristIn);
		Command intake = new IntakeSetInCommand(s.intakeSubsystem).until(s.intakeSubsystem::isSecured);
		eventMap.put("ScoreBottom", score);
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		eventMap.put("WristRetract", wristIn);
		eventMap.put("WristPrescore", wristPrescore);
		eventMap.put("IntakeOut", intakeOut);
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
}
