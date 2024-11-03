// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.cfg.ConstructorDetector.SingleArgConstructor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ShooterConfiguration;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootAmpSide extends Command {
  /** Creates a new SimpleShootCmd. */
  ArmSubsystem s_ArmSubsystem = ArmSubsystem.getInstance();
  ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
  IntakeSubsystem s_IntakeSubsystem = IntakeSubsystem.getInstance();
  private boolean hasFired = false;
  private double fireShotTimeStamp = 0;
  public SimpleShootAmpSide() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem, s_ShooterSubsystem, s_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fireShotTimeStamp = 0;
    hasFired = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            // shooterConfigurations.put(keys.get(1).get(2), new ShooterConfiguration(Pair.of(3700d,3900d), 68d, 0d));
    s_ArmSubsystem.driveToGoal(71 + ArmConstants.armDegreesOffset); //TODO: change the 71
    s_ShooterSubsystem.spinToVelocity(Pair.of(3550d,4000d)); //TODO:change both of these
    if(s_ArmSubsystem.isAtGoal() && s_ShooterSubsystem.isAtSetpoint() || hasFired)
    {
      if(!hasFired) {
        hasFired = true;
        fireShotTimeStamp = Timer.getFPGATimestamp();
      } 
      s_IntakeSubsystem.feedShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ArmSubsystem.stop();
    s_IntakeSubsystem.stop();
    s_ShooterSubsystem.stop();
    s_IntakeSubsystem.manuelShootNotesEndMethod();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
        if(hasFired && currentTime-fireShotTimeStamp > ShooterConstants.fireTime) {
            
            return true;
        }
    return false;
  }
}
