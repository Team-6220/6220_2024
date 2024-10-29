// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIdleCommand extends Command {
  /** Creates a new ShooterIdleCommand. */
  final ShooterSubsystem s_ShooterSubsystem;
  final IntakeSubsystem s_IntakeSubsystem;
  public ShooterIdleCommand() {
    s_ShooterSubsystem = ShooterSubsystem.getInstance();
    s_IntakeSubsystem = IntakeSubsystem.getInstance();
    addRequirements(s_ShooterSubsystem);
  }

  @Override
  public void execute() {
    if(s_IntakeSubsystem.noteReady() || !s_IntakeSubsystem.getNoteInIntake()) {
      s_ShooterSubsystem.spinManually(ShooterConstants.idleOutput);
    } else {
      s_ShooterSubsystem.spinManually(0);
    }
  }

}
