package frc.team2412.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.TestSubsystem;

public class TestForwardCommand extends CommandBase{
    // vars
    private TestSubsystem testSubsystem;

    // consturcter
    public TestForwardCommand(TestSubsystem testSubsystem) {
        this.testSubsystem = testSubsystem;

        addRequirements(testSubsystem);
    }

    // methods
    @Override
    public void initialize() {
        testSubsystem.setMotorForward();
    }

    @Override
    public void execute() {
        //loops set of code on robot
    }

    @Override
    public void end(boolean interrupted) {
        //runs when command stops
    }

    @Override
    public boolean isFinished() {
        return true;
    }



}
