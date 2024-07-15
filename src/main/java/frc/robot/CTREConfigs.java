package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.ShooterConstants;

public final class CTREConfigs {
    // public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public TalonFXConfiguration shooterAConfig = new TalonFXConfiguration();
    public TalonFXConfiguration shooterBConfig = new TalonFXConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        //swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        // swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.angleCurrentThreshold;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.angleCurrentThresholdTime;

        /* PID Config */
        // swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
        // swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
        // swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;

        /** Shooter Motor A, Green, Configuration */
         /* Motor Inverts and Neutral Mode */
        shooterAConfig.MotorOutput.Inverted = ShooterConstants.motorAInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        shooterAConfig.MotorOutput.NeutralMode = ShooterConstants.MOTOR_A_NEUTRAL_MODE_VALUE;

        /* Gear Ratio Config */
        // shooterAConfig.Feedback.SensorToMechanismRatio = ;

        /* Current Limiting */
        shooterAConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterAEnableCurrentLimit;
        shooterAConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterACurrentLimit;
        shooterAConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.shooterACurrentThreshold;
        shooterAConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.shooterACurrentThresholdTime;

        /* PID Config */
        // shooterAConfig.Slot0.kP = ;
        // shooterAConfig.Slot0.kI = ;
        // shooterAConfig.Slot0.kD = ;

        /* Open and Closed Loop Ramping */
        // shooterAConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ;
        // shooterAConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ;

        // shooterAConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ;
        // shooterAConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ;

        /** Shooter Motor B, Red/orange Configuration */
         /* Motor Inverts and Neutral Mode */
        shooterBConfig.MotorOutput.Inverted = Constants.ShooterConstants.motorBInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        shooterBConfig.MotorOutput.NeutralMode = Constants.ShooterConstants.MOTOR_B_NEUTRAL_MODE_VALUE;

        /* Gear Ratio Config */
        // shooterBConfig.Feedback.SensorToMechanismRatio = ;

        /* Current Limiting */
        shooterBConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterBEnableCurrentLimit;
        shooterBConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterBCurrentLimit;
        shooterBConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.shooterBCurrentThreshold;
        shooterBConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.shooterBCurrentThresholdTime;

        /* PID Config */
        // shooterBConfig.Slot0.kP = ;
        // shooterBConfig.Slot0.kI = ;
        // shooterBConfig.Slot0.kD = ;

        /* Open and Closed Loop Ramping */
        // shooterBConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ;
        // shooterBConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ;

        // shooterBConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ;
        // shooterBConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ;
    }
}