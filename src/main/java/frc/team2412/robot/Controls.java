package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.*;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;

    public Controls(Subsystems s) {
        driveController = new XboxController(CONTROLLER_PORT);

        if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
            bindDrivebaseControls();
        }
    }

    public void bindDrivebaseControls() {

    }
}
