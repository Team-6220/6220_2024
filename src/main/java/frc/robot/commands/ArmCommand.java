// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCommand extends Command {
  private final ArmSubsystem armSubsystem;
  //A: set to bottom position
  private final Supplier<Boolean> aButton;
  //Y: set to precomp resting position
  private final Supplier<Boolean> yButton;
  //X: intake a thingy
  //RTrigger: shoot speaker
  //RBumper: line up arm for speaker
  private final Supplier<Boolean> rBumper;
  //LTrigger: shoot amp
  //LBumper: line up arm for amp
  private final Supplier<Boolean> lBumper;
  //RJoyButton/LJoyButton: simultaenously press to emergency stop
  private final Supplier<Boolean> rJB, lJB;
  
  //Creates a new ArmCommand
  public ArmCommand(Supplier<Boolean> aB, Supplier<Boolean> yB, Supplier<Boolean> lBump,Supplier<Boolean> rBump, Supplier<Boolean> lJB, Supplier<Boolean> rJB) {
    this.armSubsystem = ArmSubsystem.getInstance();
    this.aButton = aB;
    this.yButton = yB;
    this.rBumper = rBump;
    this.lBumper = lBump;
    this.rJB = rJB;
    this.lJB = lJB;
    addRequirements(armSubsystem);
  }
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}