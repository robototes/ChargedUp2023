package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class TestSubsystem extends SubsystemBase {

    // CONSTANT VARIABLES

    private final double MOTOR_SPEED = 0.5;
    private final double MAX_ANGLE = 60;

    // INSTANCE VARIABLES
    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder motorEncoder;

    // CONSTRUCTOR
    //      Allows you to instantiate a class (make an object)
    //      Parameters: instance variables
    //      Sets the value instance variables
    public TestSubsystem() {


        // hi guys
        // Initialize Nessescary Variables

        this.motor = new CANSparkMax(60, MotorType.kBrushless);
        this.motorEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

        // Configure Hardware
        motor.setIdleMode(IdleMode.kBrake);

    }

    // GETTER METHODS

    // gives us the motor's current speed
    public double getSpeed() {
        return motor.get();
    }

    // gives us the motor's current angle
    public double getAngle() {
        return motorEncoder.getPosition();
    }


    // SETTER METHODS

    public void setMotorForward() {
        motor.set(MOTOR_SPEED);
    }

    public void setMotorReverse() {
        motor.set(-MOTOR_SPEED);
    }

    public void setMotor(double speed) {
        motor.set(speed);
        }

    public void periodic() {
        // constantly print the current angle
        System.out.println(getAngle());
    }

}
