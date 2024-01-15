package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private static final ArmSubsystem INSTANCE = null;

    private ArmSubsystem() {

    }

    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            return new ArmSubsystem();
        }
        return INSTANCE;
    }
}
