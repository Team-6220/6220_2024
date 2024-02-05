package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
public class ShooterSubsystem extends SubsystemBase{

    private static ShooterSubsystem INSTANCE = null;
    private TalonFX shooterMotorA;
    private TalonFX shooterMotorB;

    private ShooterSubsystem() {
      shooterMotorA = new TalonFX(ShooterConstants.shooterMotorAID);
      shooterMotorB = new TalonFX(ShooterConstants.shooterMotorBID);
    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

}
