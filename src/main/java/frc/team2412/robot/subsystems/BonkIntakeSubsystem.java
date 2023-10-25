package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team2412.robot.Hardware;

public class BonkIntakeSubsystem {

    private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;

    public BonkIntakeSubsystem() {
        wristMotor = new CANSparkMax(Hardware.BONK_INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        intakeMotor1 = new CANSparkMax(Hardware.BONK_INTAKE_MOTOR_1, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(Hardware.BONK_INTAKE_MOTOR_2, MotorType.kBrushless);

        resetMotors();
    }

    /**
     * Configure and reset motors
     */
    public void resetMotors() {
        wristMotor.restoreFactoryDefaults();
        intakeMotor1.restoreFactoryDefaults();
        intakeMotor2.restoreFactoryDefaults();

        wristMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor1.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setIdleMode(IdleMode.kBrake);

        wristMotor.setSmartCurrentLimit(20);
        intakeMotor1.setSmartCurrentLimit(20);
        intakeMotor2.setSmartCurrentLimit(20);

        wristMotor.burnFlash();
        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
    }
}
