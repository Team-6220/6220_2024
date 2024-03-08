package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeIdleCommand;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    private final CANSparkMax intakeMotor;
    private final RelativeEncoder encoder;

    private final DigitalInput frontBreakBeam;
    private final DigitalInput backBreakBeam;

    private final PIDController m_Controller;

    private boolean firing;

    private boolean noteInIntake;
    private boolean noteAtBack;

    private final TunableNumber Kp = new TunableNumber("Intake Kp", IntakeConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Intake Ki", IntakeConstants.kI);
    private final TunableNumber Kd = new TunableNumber("Intake kD", IntakeConstants.kD);
    private final TunableNumber holdingPosition = new TunableNumber("IntakeHoldingPose", IntakeConstants.holdingPosition);
    private IntakeSubsystem() {
        intakeMotor  = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        frontBreakBeam = new DigitalInput(IntakeConstants.frontBreakBeamPort);
        backBreakBeam = new DigitalInput(IntakeConstants.backBreakBeamPort);
        encoder = intakeMotor.getEncoder();
        m_Controller = new PIDController(Kp.get(), Ki.get(), Kd.get());
        m_Controller.setTolerance(.2);
    }

    public void simpleDrive(boolean reversed, double speed){
        speed = reversed ? speed * -1 : speed;
        intakeMotor.set(speed);
    }

    // public void driveToIntake(){
    //     if(!getFrontBeam() && !getBackBeam()) {
    //         //intakeMotor.set(IntakeConstants.intakeSpeed);
    //         intakeMotor.set(0);
    //     } else if(getFrontBeam() && !getBackBeam()) {
    //         intakeMotor.set(IntakeConstants.intakeSpeed*.1);
    //         noteInIntake = true;
    //     } else if(getBackBeam() && getFrontBeam()) {
    //         intakeMotor.set(0);
    //     } else{
    //         intakeMotor.set(0);
    //     }
    // }

    public void feedShooter() {
        simpleDrive(false, IntakeConstants.ejectSpeedSpeaker);
        noteInIntake = false;
        noteAtBack = false;
        firing = true;
    }

    public void feedAmp() {
        simpleDrive(false, IntakeConstants.ejectSpeedAmp);
        noteInIntake = false;
        noteAtBack = false;
        firing = true;
    }

    public void feedIntake() {
        if(!noteInIntake) {
            simpleDrive(false, IntakeConstants.intakeSpeed);
        }
    }

    public void reset() {
        noteInIntake = false;
        noteAtBack = false;
        firing = false;
    }

    public void stop(){
        intakeMotor.set(0);
        firing = false;
    }

    public boolean noteInIntake() {
        return noteInIntake;
    }

    public boolean getFrontBeam() {
        return !frontBreakBeam.get();
    }
    public boolean getBackBeam() {
        return !backBreakBeam.get();
    }

    public void setHasNote() {
        noteInIntake = true;
        noteAtBack = true;
        encoder.setPosition(-IntakeConstants.distanceBetweenBreakBeamsInEncoderRotations);
    }

    public void driveNoteToSetpoint() {
        double output = 0;
        if(!noteAtBack && getBackBeam()) {
            noteAtBack = true;
            encoder.setPosition(0);
        }
        if(noteAtBack) {
            if(m_Controller.atSetpoint()) {
                output = 0;
                intakeMotor.set(0);
                return;
            }
            output = m_Controller.calculate(encoder.getPosition(), holdingPosition.get());

        } else {
            output = m_Controller.calculate(encoder.getPosition(), holdingPosition.get());
            if(output < IntakeConstants.minSetOutput) {
                output = IntakeConstants.minSetOutput;
            }
        }
        
        intakeMotor.set(output);

    }

    public void setFiring(boolean newValue) {
        firing = newValue;
    }

    public void newNoteDetected() {
        encoder.setPosition(IntakeConstants.distanceBetweenBreakBeamsInEncoderRotations);
        m_Controller.reset();
        noteInIntake = true;
    }

    public boolean noteReady() {
        return noteInIntake&&m_Controller.atSetpoint();
    }

    @Override
    public void periodic() {
        
        if(!noteInIntake && getFrontBeam()) {
            newNoteDetected();
        }
        if(noteInIntake && !firing) {
            driveNoteToSetpoint();
        }
        
        SmartDashboard.putBoolean("Beam Front", frontBreakBeam.get());
        SmartDashboard.putBoolean("Beam Back", backBreakBeam.get());
        SmartDashboard.putNumber("IntakePosition", encoder.getPosition());
        if(Kp.hasChanged()
        || Ki.hasChanged()
        || Kd.hasChanged())
        {
            m_Controller.setPID(Kp.get(),Ki.get(),Kd.get());
        }
    }

    

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}