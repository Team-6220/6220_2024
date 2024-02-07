package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    private final TalonFX intakeMotor;

    // private final DigitalInput intakeBreakBeam;

    private IntakeSubsystem() {
        intakeMotor  = new TalonFX(IntakeConstants.intakeMotorID);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        // intakeBreakBeam = new DigitalInput(IntakeConstants.breakBeamPort);
    }

    public void simpleDrive(boolean reversed){
        double speed = reversed ? IntakeConstants.intakeSpeed * -1 : IntakeConstants.intakeSpeed;
        intakeMotor.set(speed);
    }

    public void driveToIntake(){
        // if(!hasNote()){
            intakeMotor.set(IntakeConstants.intakeSpeed);
        // }
        // else{
            // stop();
        // }
    }

    public void hopperToShooter(){
        // if(hasNote()){
            intakeMotor.set(IntakeConstants.intakeSpeed);
        // }
        // else{
            // stop();
        // }
    }

    public void stop(){
        intakeMotor.set(0);
    }

    // public boolean hasNote(){
        // return intakeBreakBeam.get();
    // }

    @Override
    public void periodic(){
        // SmartDashboard.putBoolean("Intake Has Note", hasNote());
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}