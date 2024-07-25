// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitSwerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

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
        
        turnSparkMax.restoreFactoryDefaults();

        turnSparkMax.setCANTimeout(250);

        mNeoAngleEncoder = turnSparkMax.getEncoder(Type.kHallSensor, 42);
        
        turnSparkMax.burnFlash();

        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        driveTalon.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveTalon.getConfigurator().setPosition(0.0);
        }
}