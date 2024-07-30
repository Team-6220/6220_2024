// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitSwerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Module
{
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
}