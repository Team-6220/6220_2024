package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    private static ClimberSubsystem INSTANCE = null;

    // private final CANSparkMax climbMotorA, climbMotorB;


    private ClimberSubsystem(){
        // climbMotorA = new CANSparkMax(ClimberConstants.climberDriverLeftID, MotorType.kBrushless);
        // climbMotorB = new CANSparkMax(ClimberConstants.climberDriverRightID, MotorType.kBrushless);

        // climbMotorA.restoreFactoryDefaults();
        // climbMotorB.restoreFactoryDefaults();

        // climbMotorA.setInverted(ClimberConstants.motorAInverted);
        // climbMotorB.setInverted(ClimberConstants.motorBInverted);

        // climbMotorA.setIdleMode(IdleMode.kBrake);
        // climbMotorB.setIdleMode(IdleMode.kBrake);

        // climbMotorB.follow(climbMotorA, true);
        // climbMotorA.burnFlash();
        // climbMotorB.burnFlash();
    }


    public void simpleDriveRight(double speed){
        if(speed > 0.5){
            speed = 0.5;
        }
        else if (speed < -0.5){
            speed = -0.5;
        }
        // climbMotorA.set(speed);
        // System.out.println(speed);
    }
    public void simpleDriveLeft(double speed){
        if(speed > 0.5){
            speed = 0.5;
        }
        else if (speed < -0.5){
            speed = -0.5;
        }
        // climbMotorB.set(speed);
        // System.out.println(speed);
    }

    public static ClimberSubsystem getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new ClimberSubsystem();
        }
        return INSTANCE;
    }
    
}