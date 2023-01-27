package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    // CONSTANTS
    public static class IntakeConstants {
        // speeds
        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = 0.3;

        public static final double INTAKE_CUBE_DISTANCE = 0;
        public static final double INTAKE_CONE_DISTANCE = 0;
        public static final double INTAKE_CUBE_COLOR = 0;
        public static final double INTAKE_CONE_COLOR = 141;

        // enums
        public static enum GamePieceType {
            CUBE, CONE, NONE;

            /*
             * for reference hi cammy eggy
             * String example;
             *
             * GamePieceType(String example) {
             * this.example = example;
             * }
             */

        }

        // public final cone;
        // public final cube;
        // public final nothing;

    }

    private final CANSparkMax motor;
    // private final ColorSensorV3 colorSensor;
    // private final distanceSensorIDK distanceSensor;

    // CONSTRUCTOR
    public IntakeSubsystem() {
        motor = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
        // colorSensor = new ColorSensorV3(INTAKE_COLOR_SENSOR); //to find I2C port
        // distanceSensor = new ();

        motor.setIdleMode(IdleMode.kBrake);
    }   

    // METHODS
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getSpeed() {
        return motor.get();
    }

    public void intakeIn() {
        setSpeed(INTAKE_IN_SPEED);

    }

    public void intakeOut() {
        setSpeed(INTAKE_OUT_SPEED);

    }

    public void intakeStop() {
        setSpeed(0);

    }

    public GamePieceType detectType() {
        // if () {
        //     return GamePieceType.CUBE; 
        // }
        // else if () {
        //     return GamePieceType.CONE; 
        // }

        return GamePieceType.NONE;
    }

    public boolean isSecured() {
        return false;
    } 

    public void ledStrip(boolean buttonPressed) {
    }

    public void rumble() {

    }

}
