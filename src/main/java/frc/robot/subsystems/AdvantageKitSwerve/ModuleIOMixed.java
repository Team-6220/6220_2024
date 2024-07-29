// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class ModuleIOMixed implements ModuleIO {
    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;

    private final CANcoder cancoder;

    private final RelativeEncoder mNeoAngleEncoder;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final Rotation2d absoluteEncoderOffset;
        private final boolean isTurnMotorInverted = SwerveConstants.angleMotorInvert;
    public ModuleIOMixed(int index)
    {
        switch(index)
        {
            case 0:
            driveTalon = new TalonFX(SwerveConstants.Mod0.driveMotorID);
            turnSparkMax = new CANSparkMax(SwerveConstants.Mod0.angleMotorID, MotorType.kBrushless);
            cancoder = new CANcoder(SwerveConstants.Mod0.canCoderID);
            absoluteEncoderOffset = SwerveConstants.Mod0.angleOffset;
            break;
            case 1:
            driveTalon = new TalonFX(SwerveConstants.Mod1.driveMotorID);
            turnSparkMax = new CANSparkMax(SwerveConstants.Mod1.angleMotorID, MotorType.kBrushless);
            cancoder = new CANcoder(SwerveConstants.Mod1.canCoderID);
            absoluteEncoderOffset = SwerveConstants.Mod1.angleOffset;
            break;
            case 2:
            driveTalon = new TalonFX(SwerveConstants.Mod2.driveMotorID);
            turnSparkMax = new CANSparkMax(SwerveConstants.Mod2.angleMotorID, MotorType.kBrushless);
            cancoder = new CANcoder(SwerveConstants.Mod2.canCoderID);
            absoluteEncoderOffset = SwerveConstants.Mod2.angleOffset;
            break;
            case 3:
            driveTalon = new TalonFX(SwerveConstants.Mod3.driveMotorID);
            turnSparkMax = new CANSparkMax(SwerveConstants.Mod3.angleMotorID, MotorType.kBrushless);
            cancoder = new CANcoder(SwerveConstants.Mod3.canCoderID);
            absoluteEncoderOffset = SwerveConstants.Mod3.angleOffset;
            break;
            default:
            throw new RuntimeException("Invalid module index");
        }

        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, drivePosition); // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent);
        driveTalon.optimizeBusUtilization();



        
        turnSparkMax.restoreFactoryDefaults();

        turnSparkMax.setCANTimeout(250);

        mNeoAngleEncoder = turnSparkMax.getEncoder(Type.kHallSensor, 42);
        
        turnSparkMax.burnFlash();

        
 

        driveTalon.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveTalon.getConfigurator().setPosition(0.0);
        }

        @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / SwerveConstants.driveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / SwerveConstants.driveGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / SwerveConstants.angleGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / SwerveConstants.angleGearRatio;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}