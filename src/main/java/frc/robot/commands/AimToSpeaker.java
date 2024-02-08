// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class AimToSpeaker extends Command {
  private TurnToHeading turnToHeading;
  private VisionSubsystem s_VisionSubsystem;
  private boolean hasSeenTarget;

  private Swerve s_Swerve;    
    
    public AimToSpeaker(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        //addRequirements(s_Swerve);
        s_VisionSubsystem = VisionSubsystem.getInstance();
        hasSeenTarget = false;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //This is done so that the starting position of the heading command is the same as the bot so the profiled pid loop will track smoothly
    //We may not need this
    turnToHeading = new TurnToHeading(s_Swerve, s_Swerve.getHeading().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newHeading;
    if(!hasSeenTarget && s_VisionSubsystem.hasTarget()) {
      newHeading = s_Swerve.getHeading().getDegrees() - s_VisionSubsystem.getSteeringOffset();
    } else {
      //Add a more sophisticated system
      newHeading = s_Swerve.getHeadingToSpeaker();;
    }
    turnToHeading.setHeading(newHeading);
    turnToHeading.execute();
    SmartDashboard.putBoolean("Is Facing Speaker", isFacingSpeaker());
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
    return isFacingSpeaker();
  }
}
