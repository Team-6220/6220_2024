// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIdleCommand extends Command {
  /** Creates a new ShooterIdleCommand. */
  ShooterSubsystem s_ShooterSubsystem;
  public ShooterIdleCommand() {
    s_ShooterSubsystem = ShooterSubsystem.getInstance();
    addRequirements(s_ShooterSubsystem);
  }

  @Override
  public void execute() {
    s_ShooterSubsystem.spinManually(ShooterConstants.idleOutput);
  }

}
