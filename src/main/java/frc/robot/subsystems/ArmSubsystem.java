package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;


public class ArmSubsystem extends SubsystemBase{
    private static final ArmSubsystem INSTANCE = null; //Created so that only 1 instance of arm subsystem is 
    // created at all time. Think of it as a "static" call to the subsystem where you can get static variables

    private final CANSparkMax armMotorA, armMotorB;
    
    private ArmSubsystem() {
        this.armMotorA = new CANSparkMax(Constants.ArmConstants.armMotorAID, MotorType.kBrushless);
        this.armMotorB = new CANSparkMax(Constants.ArmConstants.armMotorBID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
         // This method will be called once per scheduler run
    }

    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            return new ArmSubsystem();
        }
        return INSTANCE;
    }
}
