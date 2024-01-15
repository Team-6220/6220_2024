package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private static final ShooterSubsystem INSTANCE = null;

    private ShooterSubsystem() {

    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            return new ShooterSubsystem();
        }
        return INSTANCE;
    }

}
