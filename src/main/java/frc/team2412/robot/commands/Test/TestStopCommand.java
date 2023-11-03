package frc.team2412.robot.commands.Test;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.TestSubsystem;

public class TestStopCommand extends CommandBase{
	private TestSubsystem testSubsystem;

	public TestStopCommand(TestSubsystem testSubsystem) {
        this.testSubsystem = testSubsystem;

        addRequirements(testSubsystem);
    }

	@Override
    public void initialize() {
        testSubsystem.setMotor(0);
    }

	@Override
    public boolean isFinished() {
        return true;
    }
}
