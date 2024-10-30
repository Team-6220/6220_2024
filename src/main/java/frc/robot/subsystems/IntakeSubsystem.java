package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeIdleCommand;
import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;
public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem INSTANCE = null;

    public final CANSparkMax intakeMotor;
    private final RelativeEncoder encoder;

    //private final DigitalInput frontBreakBeam;
    //private final DigitalInput backBreakBeam;
    private final TimeOfFlight frontToF = new TimeOfFlight(1);
    private final TimeOfFlight backToF = new TimeOfFlight(0);
    private final PIDController m_Controller;
    private final PIDController m_VelocityController;
    private SimpleMotorFeedforward m_Feedforward; 

    private boolean firing;

    private boolean noteInIntake;
    private boolean noteAtBack;
    private boolean noteSecure;
    private boolean hasExited;

    private final TunableNumber Kp = new TunableNumber("Intake Kp", IntakeConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Intake Ki", IntakeConstants.kI);
    //private final TunableNumber Kd = new TunableNumber("Intake kD", IntakeConstants.kD);
    private final TunableNumber holdingPosition = new TunableNumber("IntakeHoldingPose", IntakeConstants.holdingPosition);
    private final TunableNumber intakeSpeed = new TunableNumber("IntakeSpeed", IntakeConstants.intakeRPMSpeed);

    private final TunableNumber Ks = new TunableNumber("IntakeKs", IntakeConstants.Ks);
    private final TunableNumber Kv = new TunableNumber("IntakeKv", IntakeConstants.Kv);
    private final TunableNumber KpVel = new TunableNumber("IntakeKpVel", IntakeConstants.velocityPIDConstants[0]);

    private IntakeSubsystem() {
        frontToF.setRangingMode(RangingMode.Short, 24);
        backToF.setRangingMode(RangingMode.Short, 24);
        intakeMotor  = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
        // frontBreakBeam = new DigitalInput(IntakeConstants.frontBreakBeamPort);
        // backBreakBeam = new DigitalInput(IntakeConstants.backBreakBeamPort);
        encoder = intakeMotor.getEncoder();
        m_Controller = new PIDController(Kp.get(), Ki.get(), IntakeConstants.kD);
        m_VelocityController = new PIDController(IntakeConstants.velocityPIDConstants[0], IntakeConstants.velocityPIDConstants[1], IntakeConstants.velocityPIDConstants[2]);
        m_Feedforward = new SimpleMotorFeedforward(Ks.get(), Kv.get());
        m_Controller.setTolerance(.2);
    }

    public void simpleDrive(boolean reversed, double speed){
        speed = reversed ? speed * -1 : speed;
        intakeMotor.set(speed);
    }

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

            simpleDrive(false, IntakeConstants.intakeSpeed);
        
    }

    public void reset() {
        noteInIntake = false;
        noteAtBack = false;
        noteSecure = false;
        hasExited = false;
        firing = false;
        m_Controller.reset();
        m_VelocityController.reset();
        encoder.setPosition(0);
    }

    public void stop(){
        intakeMotor.set(0);
        firing = false;
    }

    public boolean noteInIntake() {
        return noteInIntake;
    }

    public boolean getFrontBeam() {
        return frontToF.getRange()<95;
    }
    public boolean getBackBeam() {
        return backToF.getRange()<95;
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
            if(noteSecure) {
                intakeMotor.set(0);
                return;
            } else {
                if(hasExited) {
                    output = .065;
                } else {
                    output = -.15;
                }
                
            }
            if(!getBackBeam()) {
                hasExited = true;
            }
            if(hasExited && getBackBeam()) {
                noteSecure = true;
            }
        } else {
            output = m_Feedforward.calculate(intakeSpeed.get()) + m_VelocityController.calculate(encoder.getVelocity(), intakeSpeed.get());
        }
        
        intakeMotor.set(output);

    }
    // private void driveWithBackup() {
    //     double output = 0;
    //     if(!noteAtBack && !getFrontBeam()) {
    //         noteAtBack = true;
    //         hasExited = true;
    //         encoder.setPosition(0);
    //     }
    //     if(noteAtBack) {
    //         if(noteSecure) {
    //             intakeMotor.set(0);
    //             return;
    //         } else if(hasExited && !getFrontBeam()) {
    //              output = .1;
    //         }
    //         if(!noteSecure && getFrontBeam()) {
    //             noteSecure = true;
    //         }
    //     } else {
    //         output = m_Feedforward.calculate(intakeSpeed.get()) + m_VelocityController.calculate(encoder.getVelocity(), intakeSpeed.get());
    //     }
    //     intakeMotor.set(output);
    // }

    public void testRPMPID() {
        double output = 0;
        double pidOut = m_VelocityController.calculate(encoder.getVelocity(), intakeSpeed.get());
        output = m_Feedforward.calculate(intakeSpeed.get()) + pidOut;
        System.out.println("Output: " + output + " PID: " + pidOut);
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
        return noteSecure;
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Intake front beam", frontToF.getRange());
        if(!noteInIntake && getFrontBeam()) {
            newNoteDetected();
        }
        // if(IntakeConstants.backupModeCount <= 1) {
        //     if(noteInIntake && !firing) {
        //         driveNoteToSetpoint();
        //     }
        // } 
        // else if(IntakeConstants.backupModeCount <= 5) {
        //     if(noteInIntake && !firing) {
        //         driveWithBackup();
        //     }
        // }
        
        
        //SmartDashboard.putBoolean("Beam Front", frontBreakBeam.get());
        //SmartDashboard.putBoolean("Beam Back", backBreakBeam.get());
        SmartDashboard.putBoolean("FrontTOF", getFrontBeam());
        SmartDashboard.putBoolean("Back TOF", getBackBeam());
        SmartDashboard.putNumber("IntakePosition", encoder.getPosition());
        SmartDashboard.putNumber("Intake RPM", encoder.getVelocity());
        if(Kp.hasChanged()
        || Ki.hasChanged())
        {
            m_Controller.setPID(Kp.get(),Ki.get(),IntakeConstants.kD);
        }

        if(Ks.hasChanged()
        || Kv.hasChanged()) {
            m_Feedforward = new SimpleMotorFeedforward(Ks.get(), Kv.get());
        }
        if(KpVel.hasChanged()) {
            m_VelocityController.setPID(KpVel.get(), 0 ,0);
        }
    }

    

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}