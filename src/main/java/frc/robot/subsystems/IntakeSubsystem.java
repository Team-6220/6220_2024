package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    private final TalonFX intakeMotor;

    private final DigitalInput intakeBreakBeam;
    private boolean hasNote = false;
    private boolean noteInTransit = false;

    private final PIDController intakeController;

    private final TunableNumber Kp = new TunableNumber("Intake kP", IntakeConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Intake kI", IntakeConstants.kI);
    private final TunableNumber Kd = new TunableNumber("Intake kD", IntakeConstants.kD);

    private final TunableNumber transDist = new TunableNumber("Intake Transit Distance", IntakeConstants.transitDistance);

    private IntakeSubsystem() {
        intakeMotor  = new TalonFX(IntakeConstants.intakeMotorID);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        intakeBreakBeam = new DigitalInput(IntakeConstants.breakBeamPort);
        intakeController = new PIDController(Kp.get(), Ki.get(), Kd.get());
    }

    public void simpleDrive(boolean reversed, double speed){
        speed = reversed ? speed * -1 : speed;
        intakeMotor.set(speed);
    }

    public void driveToIntake(){
        if(!noteInBeam() && !hasNote && !noteInTransit){
            intakeMotor.set(IntakeConstants.intakeSpeed);
        } else if (noteInBeam() && !hasNote && !noteInTransit) {
            noteInTransit = true;
            intakeMotor.setPosition(0);
            
        } else if(noteInTransit) {
            intakeMotor.set(intakeController.calculate(intakeMotor.getPosition().getValueAsDouble(), transDist.get()));
        } else {
            stop();
        }
    }

    public void feedShooter() {
        simpleDrive(true, IntakeConstants.ejectSpeedSpeaker);
    }

    public void feedAmp()
    {
        simpleDrive(true, IntakeConstants.ejectSpeedAmp);
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