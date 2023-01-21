package frc.team2412.robot.subsystems;

import frc.team2412.robot.Hardware;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class IntakeSubsystem extends SubsystemBase {
    // CONSTANTS
    public static class IntakeConstants {
        //speeds
        public static final double INTAKE_IN_SPEED;
        public static final double INTAKE_OUT_SPEED;
            
        public static final double INTAKE_CUBE_DISTANCE;
        public static final double INTAKE_CONE_DISTANCE;
        public static final double INTAKE_CUBE_COLOR;
        public static final double INTAKE_CONE_COLOR;

        // enums
        public static enum GamePieceType {}
        // public final cone;
        // public final cube;
        // public final nothing;
        
    }

    private final CANSparkMax motor;
    private final ColorSensorV3 colorSensor;
    // private final distancecSensorIDK distanceSensor;

    // CONSTRUCTOR
    public IntakeSubsystem() {
        //motor = new CANSparkMax(INTAKE_MOTOR , MotorType.kBrushless);
        //colorSensor = new ColorSensorV3(0); //to find I2C port
        //distanceSensor = new ();


        //motor.setIdleMode(kBrake);
    }

    // METHODS
    public void setSpeed(double speed) {
        //motor.setSpeed(speed);
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
    
    // public GamePieceType detectType() {
    //     return INTAKE_TYPE;
    // }
    
    public boolean isSecured() {
        return false; 
    }
    
    public void ledStrip(boolean buttonPressed) {
    }
   
}

