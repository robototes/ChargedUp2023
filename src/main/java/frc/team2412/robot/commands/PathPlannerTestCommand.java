package frc.team2412.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class PathPlannerTestCommand extends CommandBase {
    
    DrivebaseSubsystem drivebaseSubsystem;

    public PathPlannerTestCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;
    }

    @Override
    public void initialize() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(1, 0.3));

        Command fullAuto = Robot.autoBuilder.fullAuto(pathGroup);

        fullAuto.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
