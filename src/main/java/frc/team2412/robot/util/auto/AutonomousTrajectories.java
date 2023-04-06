package frc.team2412.robot.util.auto;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.arm.SetFullArmCommand;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.arm.SetWristCommand.WristPosition;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import frc.team2412.robot.commands.intake.IntakeSetFastOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	private static final Subsystems s = Robot.getInstance().subsystems;

	public static Command getAutoPathByName(String name) {
		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup(name, new PathConstraints(2.0, 2.0));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		if (!(s.intakeSubsystem == null) && !(s.armSubsystem == null)) {
			SetWristCommand wristOut = new SetWristCommand(s.armSubsystem, WristPosition.WRIST_SCORE);
			SetWristCommand wristPrescore =
					new SetWristCommand(s.armSubsystem, WristPosition.WRIST_PRESCORE);
			Command intakeOut = new IntakeSetOutCommand(s.intakeSubsystem).withTimeout(0.5);
			Command intakeFastOut = new IntakeSetFastOutCommand(s.intakeSubsystem).withTimeout(0.5);
			Command intakeIn = new IntakeSetInCommand(s.intakeSubsystem).withTimeout(0.5);
			SetWristCommand wristIn = new SetWristCommand(s.armSubsystem, WristPosition.WRIST_RETRACT);
			SequentialCommandGroup scoreBottom =
					new SequentialCommandGroup(intakeIn, wristPrescore, intakeOut.withTimeout(1.5), wristIn);
			Command armLow = new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_PRESCORE);
			Command armMid = new SetFullArmCommand(s.armSubsystem, ARM_MIDDLE_POSITION, WRIST_PRESCORE);
			Command armHigh = new SetFullArmCommand(s.armSubsystem, ARM_HIGH_POSITION, WRIST_PRESCORE);
			Command stow =
					new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WristPosition.WRIST_RETRACT, 0.3);
			Command scoreHigh =
					new SequentialCommandGroup(
							new SetFullArmCommand(s.armSubsystem, ARM_HIGH_POSITION, WristPosition.WRIST_SCORE),
							new WaitCommand(0.5),
							new IntakeSetOutCommand(s.intakeSubsystem).withTimeout(0.3));

			eventMap.put("ScoreBottom", scoreBottom);
			eventMap.put("ScoreHigh", scoreHigh);

			eventMap.put("WristRetract", wristIn);
			eventMap.put("WristPrescore", wristPrescore);
			eventMap.put("IntakeOut", intakeOut);
			eventMap.put("IntakeFastOut", intakeFastOut);
			eventMap.put("IntakeIn", intakeIn);
			eventMap.put("ArmHigh", armHigh);
			eventMap.put("WristOut", wristOut);
			eventMap.put("ArmLow", armLow);
			eventMap.put("ArmMid", armMid);
			eventMap.put("Stow", stow);
			eventMap.put("Wait", new WaitCommand(0.5));
		}
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
}
