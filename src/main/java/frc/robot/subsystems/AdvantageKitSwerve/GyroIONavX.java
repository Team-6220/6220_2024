// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitSwerve;

import com.ctre.phoenix6.StatusSignal;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class GyroIONavX implements GyroIO{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    
    public GyroIONavX()
    {
        gyro.setAngleAdjustment(0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = gyro.getWorldLinearAccelZ();
    }
}
