// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;

public class CenterNoteCmd extends Command {
  /** Creates a new CenterNoteCmd. */
  ArmSubsystem s_ArmSubsystem = ArmSubsystem.getInstance();
  IntakeSubsystem s_IntakeSubsystem = IntakeSubsystem.getInstance();
  PhotonVisionSubsystem photon = PhotonVisionSubsystem.getInstance();
  Swerve s_Swerve;

  private boolean isFinished = false;
  public CenterNoteCmd(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.setIsAuto(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(AutoConstants.currentCenterNotePos > AutoConstants.centerNoteMax || AutoConstants.currentCenterNotePos < AutoConstants.centerNoteMin)
    {
      isFinished = true;
    }
    // AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.currentCenterNotePos], AutoConstants.pathConstraints); //Enable this line only if the commands.race doesn't work, which most likely mean that something with intake buffer is wrong.
    Commands.race(AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.currentCenterNotePos], AutoConstants.pathConstraints, 0, 1.5), new IntakeBuffer());
    if(!photon.getHasTargets())
    {
      AutoConstants.currentCenterNotePos += AutoConstants.centernoteIncrementVal;
    }
    else if(photon.getHasTargets())
    {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
