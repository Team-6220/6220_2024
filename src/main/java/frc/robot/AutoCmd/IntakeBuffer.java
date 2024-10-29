// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.PhotonvisionCalculations;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class IntakeBuffer extends Command {
  /** Creates a new IntakeBuffer. */
  ArmSubsystem arm = ArmSubsystem.getInstance();
  IntakeSubsystem intake = IntakeSubsystem.getInstance();
  PhotonVisionSubsystem photonVisionSubsystem = PhotonVisionSubsystem.getInstance();
  public IntakeBuffer() {
    addRequirements(arm, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.driveToGoal(ArmConstants.intakeSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(photonVisionSubsystem.getHasTargets())
    {
      // System.out.println("Has target!");
      return false;
    }
    return false;
  }
}
