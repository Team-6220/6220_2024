package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private static final ClimberSubsystem INSTANCE = null;

    private ClimberSubsystem(){

    }

    public static ClimberSubsystem getInstance(){
        if(INSTANCE == null) {
            return new ClimberSubsystem();
        }
        return INSTANCE;
    }
    
}
