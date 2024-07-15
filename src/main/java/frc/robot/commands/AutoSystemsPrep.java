// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ShooterConfiguration;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.blinkin;


public class AutoSystemsPrep extends Command {
  /** Creates a new IdleCommand. */
  ArmSubsystem s_ArmSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  Swerve s_Swerve;
  Pose2d goalPose;
  ShooterConfiguration shooterConfig = null;

  public AutoSystemsPrep(Pose2d goalPose, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    s_ArmSubsystem = ArmSubsystem.getInstance();
    s_ShooterSubsystem = ShooterSubsystem.getInstance();
    this.goalPose = goalPose;
    try {
      shooterConfig = ShooterConfiguration.getShooterConfiguration(goalPose);
    } catch(Exception e) {
      System.out.println(e);
    }
    addRequirements(s_ArmSubsystem, s_ShooterSubsystem);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterConfig != null) {
      if(Math.hypot(s_Swerve.getPose().getX() - goalPose.getX(), s_Swerve.getPose().getY()-goalPose.getY()) < 1) {
        s_ArmSubsystem.driveToGoal(shooterConfig.getArmAngle());
        s_ShooterSubsystem.spinToVelocity(shooterConfig.getVelocities());
      } else {
        s_ArmSubsystem.driveToGoal(ArmConstants.restingSetpoint);
        s_ShooterSubsystem.stop();
      }
    } else {
      s_ArmSubsystem.driveToGoal(ArmConstants.restingSetpoint);
    }
    
  }


}
