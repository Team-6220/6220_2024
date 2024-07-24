// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitSwerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class ModuleIOMixed implements ModuleIO {
    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;

    private final 

    private final CANcoder cancoder;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;
    public ModuleIOMixed(int index)
    {
        switch(index)
        {
            case 0:
            driveTalon = new TalonFX(SwerveConstants.Mod0.driveMotorID);
            turnSparkMax = new CANSparkMax(SwerveConstants.Mod0.angleMotorID, MotorType.kBrushless);
            cancoder = new CANcoder(SwerveConstants.Mod0.canCoderID);
        }
    }
}
