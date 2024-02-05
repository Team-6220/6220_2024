package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private static ClimberSubsystem INSTANCE = null;

    private ClimberSubsystem(){

    }

    public static ClimberSubsystem getInstance(){
        if(INSTANCE == null) {
            INSTANCE = new ClimberSubsystem();
        }
        return INSTANCE;
    }
    
}