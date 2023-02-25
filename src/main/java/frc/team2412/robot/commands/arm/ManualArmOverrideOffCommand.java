package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;

public class ManualArmOverrideOffCommand extends CommandBase {

	private ArmSubsystem armSubsystem;

	public ManualArmOverrideOffCommand(ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		armSubsystem.setManualOverride(false);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
