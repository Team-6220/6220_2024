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
import edu.wpi.first.wpilibj.smartdashboard.*;

// import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

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

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

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
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleCurrentThreshold);
        mAngleMotor.setIdleMode(SwerveConstants.angleNeutralMode);

        /* Angle Motor PID Config */
        mAngleController = mAngleMotor.getPIDController();
        mAngleController.setP(Constants.SwerveConstants.angleKP);
        mAngleController.setI(Constants.SwerveConstants.angleKI);
        mAngleController.setD(Constants.SwerveConstants.angleKD);

        mAngleController.setPositionPIDWrappingEnabled(true); //wraps the numbers around when it's too big. ex if the limits are 0 and 100, it will "wrap" back to 0 after it exceeds 100, vise versa
        mAngleController.setPositionPIDWrappingMinInput(0);
        mAngleController.setPositionPIDWrappingMaxInput(RevConfigs.CANCoderAngleToNeoEncoder(1));

        /* Angle Motor Encoder Config */
        mNeoAngleEncoder = mAngleMotor.getEncoder(Type.kHallSensor, 42);
        mAngleMotor.burnFlash();
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig); //motor inverted, current limits, etc. editable in constants.java. CTREConfigs.java is just a holder to organize the values
        mDriveMotor.getConfigurator().setPosition(0.0);
        if(moduleNumber == 0)
        {
            mDriveMotor.setInverted(true);
        }
        // mDriveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleController.setReference(RevConfigs.CANCoderAngleToNeoEncoder(desiredState.angle.getRotations()), ControlType.kPosition);
        overallDesiredModuleState = desiredState.angle.getDegrees();
        SmartDashboard.putNumber("Desired position mod " +moduleNumber , (desiredState.angle.getDegrees()));
        setSpeed(desiredState, isOpenLoop);
    }

    public double getDesiredState(){
        return overallDesiredModuleState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public void stopDriving()
    {
        mDriveMotor.set(0);
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    //** Points the module forward */
    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        SmartDashboard.putNumber("absolutePosition mod reset to absolute " + moduleNumber, absolutePosition);
        System.out.println("reset to absolute mod " + moduleNumber +" cancoder rotations:" + getCANcoder().getRotations() + "angleoffset.getrotations: " + angleOffset.getRotations() +"absoluteposition: " + absolutePosition, " New angle encoder:" + mNeoAngleEncoder.getPosition());
        mNeoAngleEncoder.setPosition(RevConfigs.CANCoderAngleToNeoEncoder(absolutePosition));     
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition()))
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition()))
        );
    }
}