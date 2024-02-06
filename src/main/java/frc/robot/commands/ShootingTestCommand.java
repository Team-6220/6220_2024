// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingTestCommand extends Command {

  ArmSubsystem armSubsystem;
  ShooterSubsystem shooterSubsystem;
  /** Creates a new ShootingTest. */

  public ShootingTestCommand() {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    addRequirements(armSubsystem, shooterSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.driveToGoal(65);
    shooterSubsystem.spinToVelocity(180);
  }

  @Override
  public void end(boolean interrupted){
    armSubsystem.stop();
    shooterSubsystem.stop();
  }
}
