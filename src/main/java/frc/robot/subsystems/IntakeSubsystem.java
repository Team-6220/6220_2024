package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    private final CANSparkMax intakeMotor;

    private final DigitalInput frontBreakBeam;
    private final DigitalInput backBreakBeam;

    private boolean noteInTransit;

    private final TunableNumber Kp = new TunableNumber("Intake kP", IntakeConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Intake kI", IntakeConstants.kI);
    private final TunableNumber Kd = new TunableNumber("Intake kD", IntakeConstants.kD);

    private final TunableNumber transDist = new TunableNumber("Intake Transit Distance", IntakeConstants.transitDistance);

    private IntakeSubsystem() {
        intakeMotor  = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        frontBreakBeam = new DigitalInput(IntakeConstants.frontBreakBeamPort);
        backBreakBeam = new DigitalInput(IntakeConstants.backBreakBeamPort);
    }

    public void simpleDrive(boolean reversed, double speed){
        speed = reversed ? speed * -1 : speed;
        intakeMotor.set(speed);
    }

    public void driveToIntake(){
        if(!getFrontBeam() && !getBackBeam()) {
            //intakeMotor.set(IntakeConstants.intakeSpeed);
            intakeMotor.set(0);
        } else if(getFrontBeam() && !getBackBeam()) {
            intakeMotor.set(IntakeConstants.intakeSpeed*.1);
            noteInTransit = true;
        } else if(getBackBeam() && getFrontBeam()) {
            intakeMotor.set(0);
        } else{
            intakeMotor.set(0);
        }
    }

    public void feedShooter() {
        simpleDrive(false, IntakeConstants.ejectSpeedSpeaker);
        noteInTransit = false;
    }

    public void feedAmp()
    {
        simpleDrive(false, IntakeConstants.ejectSpeedAmp);
        noteInTransit = false;
    }

    public void reset() {
        noteInTransit = false;
    }

    public void stop(){
        intakeMotor.set(0);
    }

    public boolean noteInTransit() {
        return noteInTransit;
    }

    public boolean getFrontBeam() {
        return !frontBreakBeam.get();
    }
    public boolean getBackBeam() {
        return !backBreakBeam.get();
    }

    public void setHasNote() {
        noteInTransit = true;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam Front", frontBreakBeam.get());
        SmartDashboard.putBoolean("Beam Back", backBreakBeam.get());
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}