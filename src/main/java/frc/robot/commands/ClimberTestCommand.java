// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestCommand extends Command {
  /** Creates a new ClimberTestCommand. */
  private ClimberSubsystem s_ClimberSubsystem;
  private final XboxController driver;
  public ClimberTestCommand(XboxController driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_ClimberSubsystem = ClimberSubsystem.getInstance();
    this.driver = driver;
    addRequirements(s_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driver.getLeftTriggerAxis() > 0.1) {
      s_ClimberSubsystem.simpleDrive(-(driver.getLeftTriggerAxis()-.1));
    } else if(driver.getRightTriggerAxis() > .1) {
      s_ClimberSubsystem.simpleDrive(driver.getRightTriggerAxis()-.1);
    } else{
      s_ClimberSubsystem.simpleDrive(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
