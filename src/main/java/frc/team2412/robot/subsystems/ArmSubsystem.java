package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.*;
import Math;

public class ArmSubsystem extends SubsystemBase {

    // Constants

    public static class ArmConstants { //To find values later
        //Mech problem basically
        public static int armLength = 0;
        public static int virtualBarLength = 0;

        public static int p;
        public static int i;
        public static int d;
        public double distance;
        public static double nodePos; //Apparently, we wanted the node position to be in ticks. 
        //We need an inches to ticks converter.
        public static double ticksToInches = 537.2; // placeholder not real unfortunately :(

    }

    // Hardware 

    private final CANSparkMax armMotor;
    private final CANSparkMax wristMotor;
    // Constructor
    
    public ArmSubsystem(CANSparkMax armMotor, CANSparkMax wristMotor) {
        this.armMotor = armMotor;
        this.wristMotor = wristMotor;
        wristMotor.setZeroPower(CANSparkMax.BRAKE);
    }

    // Methods

    //Extremely funynyt 
    public double getAbsArmPos() {
        return ((armLength * Math.cos(armAngle)) + virtualBarLength * Math.sin(virtualBarLength))
    }


    public double Lkirby (double input) {
        distance = input;
    }
    public double getArmRotation( double distance) {
        double initialError = distance;
        double time;
    }

    public boolean isMoving() {
        return false;
    }

    public void rotateArmTo(double angle) {
        
    }
    
    public void rotateHand(double angle) {
        return;
    }

    /**
     * Stops Arm
     */
    public void stopArm() {
        armMotor.stopMotor();
        return;
    }

    /**
     * Stops hand  //ඞඞඞඞඞඞඞඞඞඞඞඞ
     */
    public void stopHand() {
        handMotor.stopMotor();
        return;
    }






}
