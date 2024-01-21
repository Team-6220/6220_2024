package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    //private TalonFX mAngleMotor;
    private CANSparkMax mAngleMotor;
    private SparkPIDController mAngleController;
    private RelativeEncoder mNeoAngleEncoder;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private double overallDesiredModuleState;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    // private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        SmartDashboard.putString("Mod: " + moduleNumber, angleOffset.toString());
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentThreshold);
        mAngleMotor.burnFlash();

        /* Angle Motor PID Config */
        mAngleController = mAngleMotor.getPIDController();
        // mAngleController.setP(Constants.Swerve.angleKP);
        // mAngleController.setI(Constants.Swerve.angleKI);
        // mAngleController.setD(Constants.Swerve.angleKD);
        mAngleController.setP(0.5);
        mAngleController.setI(0);
        mAngleController.setD(0.15);

        mAngleController.setPositionPIDWrappingEnabled(true);
        mAngleController.setPositionPIDWrappingMinInput(0);
        mAngleController.setPositionPIDWrappingMaxInput(1);// was 2*pi, Sean said to change it

        /* Angle Motor Encoder Config */
        mNeoAngleEncoder = mAngleMotor.getEncoder(Type.kHallSensor, 42);
        mAngleMotor.burnFlash();
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleController.setReference(RevConfigs.CANCoderAngleToNeoEncoder(desiredState.angle.getRotations()), ControlType.kPosition);
        overallDesiredModuleState = desiredState.angle.getDegrees();
        // SmartDashboard.putNumber("Desired position", (desiredState.angle.getDegrees()));
        setSpeed(desiredState, isOpenLoop);
    }

    public double getDesiredState(){
        return overallDesiredModuleState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        SmartDashboard.putNumber("absolutePosition", absolutePosition);
        mNeoAngleEncoder.setPosition(RevConfigs.CANCoderAngleToNeoEncoder(absolutePosition));
        SmartDashboard.putNumber("absolutePosition degress", mNeoAngleEncoder.getPosition()*360);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition()))
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition()))
        );
    }
}