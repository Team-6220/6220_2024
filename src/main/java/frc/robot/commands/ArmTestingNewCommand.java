// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import java.util.function.Supplier;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestingNewCommand extends Command {
  private final ArmPIDSubsystem armSubsystem;

 private final Supplier<Boolean> aButton, yButton, rBumper, lBumper;
  
  private final Supplier<Double> joystick;

  /** Creates a new ArmTestingNewCommand. */
  public ArmTestingNewCommand(Supplier<Boolean> aB, Supplier<Boolean> yB, Supplier<Boolean> lBump,Supplier<Boolean> rBump, Supplier<Double> js) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = ArmPIDSubsystem.getInstance();
    this.aButton = aB;
        this.yButton = yB;
        this.rBumper = rBump;
        this.lBumper = lBump;
        this.joystick = js;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            // boolean pid = false;
        double setpoint = 0, raw_value = 0;
        if(this.aButton.get()){
            // pid = true;
            setpoint = 0;
        }else if(this.yButton.get()){
            // pid = true;
            setpoint = 85;
        }else if(this.rBumper.get()){
            // pid = true;
            setpoint = 60;
        }else if(this.lBumper.get()){
            // pid = true;
            setpoint = 40;
        }
            armSubsystem.useOutput(setpoint,0);
            System.out.println("succcesss!!!!");
        
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
