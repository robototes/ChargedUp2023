package frc.team2412.robot;

import static java.lang.Thread.sleep;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.MACAddress;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
    /**
     * Singleton Stuff
     */
    private static Robot instance = null;

    enum RobotType {
        COMPETITION, AUTOMATED_TEST, DRIVEBASE;
    }

    public static Robot getInstance() {
        if (instance == null)
            instance = new Robot();
        return instance;
    }

    private final PowerDistribution PDP;

    public Controls controls;
    public Subsystems subsystems;

    final private RobotType robotType;

    private Thread controlAuto;

    protected Robot(RobotType type) {
        System.out.println("Robot type: " + (type.equals(RobotType.AUTOMATED_TEST) ? "AutomatedTest" : "Competition"));
        instance = this;
        PDP = new PowerDistribution(Hardware.PDP_ID, ModuleType.kRev);
        robotType = type;
    }

    public double getVoltage() {
        return PDP.getVoltage();
    }

    protected Robot() {
        this(getTypeFromAddress());
    }

    public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x33, 0x9d, 0xe7);
    public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x28, 0x40, 0x82);

    private static RobotType getTypeFromAddress() {
        if (PRACTICE_ADDRESS.exists())
            return RobotType.DRIVEBASE;
        else
            return RobotType.COMPETITION;
    }

    @Override
    public void startCompetition() {
        if (!robotType.equals(RobotType.AUTOMATED_TEST)) {
            super.startCompetition();
        } else {
            try {
                super.startCompetition();
            } catch (Throwable throwable) {
                Throwable cause = throwable.getCause();
                if (cause != null) {
                    // We're about to exit, so overwriting the param is fine
                    // noinspection AssignmentToCatchBlockParameter
                    throwable = cause;
                }
                DriverStation.reportError(
                        "Unhandled exception: " + throwable.toString(), throwable.getStackTrace());

                try {
                    sleep(2000);
                } catch (InterruptedException ignored) {
                }
                java.lang.System.exit(-1);
            }
        }
    }

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.enableTelemetry(PDP);

        subsystems = new Subsystems();
        controls = new Controls(subsystems);

        Shuffleboard.startRecording();

        if (RobotBase.isReal()) {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog(), true);
        }

        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command -> System.out.println("Command initialized: " + command.getName()));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command -> System.out.println("Command interrupted: " + command.getName()));
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command -> System.out.println("Command finished: " + command.getName()));

        if (robotType.equals(RobotType.AUTOMATED_TEST)) {
            controlAuto = new Thread(() -> {
                System.out.println("Waiting two seconds for robot to finish startup");
                try {
                    sleep(2000);
                } catch (InterruptedException ignored) {
                }

                System.out.println("Enabling autonomous mode and waiting 10 seconds");
                DriverStationDataJNI.setAutonomous(true);
                DriverStationDataJNI.setEnabled(true);

                try {
                    sleep(10000);
                } catch (InterruptedException ignored) {
                }

                System.out.println("Disabling robot and waiting two seconds");
                DriverStationDataJNI.setEnabled(false);

                try {
                    sleep(2000);
                } catch (InterruptedException ignored) {
                }

                System.out.println("Ending competition");
                suppressExitWarning(true);
                endCompetition();
            });
            controlAuto.start();
        }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void robotPeriodic() {
        Logger.updateEntries();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Shuffleboard.startRecording();
    }

    @Override
    public void teleopInit() {
        Shuffleboard.startRecording();
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        PhysicsSim sim = PhysicsSim.getInstance();
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public RobotType getRobotType() {
        return robotType;
    }

    public boolean isCompetition() {
        return getRobotType() == RobotType.COMPETITION || getRobotType() == RobotType.AUTOMATED_TEST;
    }

    @Override
    public void disabledInit() {
        Shuffleboard.stopRecording();
    }
}
