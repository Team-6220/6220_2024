// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.PhotonVisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AimToClosestNoteCmd extends Command {
  /** Creates a new AimToClosestNoteCmd. */
  private TurnToHeading turnToHeading;
  private PhotonVisionSubsystem s_PhotonVisionSubsystem;

  private Swerve s_Swerve;

  public AimToClosestNoteCmd(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    s_PhotonVisionSubsystem = PhotonVisionSubsystem.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnToHeading = new TurnToHeading(s_Swerve,s_Swerve.getHeading().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newHeading;
    if(s_PhotonVisionSubsystem.getHasTargets())
    {
      newHeading = s_Swerve.getHeading().getDegrees() - s_PhotonVisionSubsystem.getTurnOffset();
    }
    else{
      newHeading = 90; //TODO: change this
    }
    turnToHeading.setHeading(newHeading);
    turnToHeading.execute();
    SmartDashboard.putBoolean("Is Facing note", isFacingSpeaker());
  }

  public boolean isFacingSpeaker() {
    if(turnToHeading.isFacingHeading()) {
      return true;
    }
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnToHeading.end(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
