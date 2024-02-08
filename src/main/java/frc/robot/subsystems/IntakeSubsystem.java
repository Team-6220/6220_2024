package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    private final TalonFX intakeMotor;

    private final DigitalInput intakeBreakBeam;
    private boolean hasNote = false;
    private boolean noteInTransit = false;

    private IntakeSubsystem() {
        intakeMotor  = new TalonFX(IntakeConstants.intakeMotorID);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        intakeBreakBeam = new DigitalInput(IntakeConstants.breakBeamPort);
    }

    public void simpleDrive(boolean reversed, double speed){
        speed = reversed ? speed * -1 : speed;
        intakeMotor.set(speed);
    }

    public void driveToIntake(){
        if(!noteInBeam() && !hasNote && !noteInTransit){
            intakeMotor.set(-IntakeConstants.intakeSpeed);
        } else if (noteInBeam() && !hasNote && !noteInTransit) {
            noteInTransit = true;
            
        } else if(noteInTransit) {
            System.out.println("Falcon Position: note in transit" );
            intakeMotor.set(-.2);
        }
    }

    public void feedShooter() {
        simpleDrive(true, IntakeConstants.ejectSpeedSpeaker);
    }

    public void hopperToShooter(){
        if(noteInBeam()){
            intakeMotor.set(IntakeConstants.intakeSpeed);
        }
        else{
            stop();
        }
    }

    public void stop(){
        intakeMotor.set(0);
    }

    public boolean noteInBeam(){
        return !intakeBreakBeam.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam In Note", noteInBeam());
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}