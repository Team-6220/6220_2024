package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private static final IntakeSubsystem INSTANCE = null;

    private IntakeSubsystem() {

    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            return new IntakeSubsystem();
        }
        return INSTANCE;
    }
}
