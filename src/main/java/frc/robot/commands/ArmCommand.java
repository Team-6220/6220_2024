// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState.State;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double armGoal;
  private final BooleanSupplier trigger;
  //Creates a new ArmCommand
  public ArmCommand(BooleanSupplier trigger) {
    this.armSubsystem = ArmSubsystem.getInstance();
    this.trigger = trigger;
    addRequirements(armSubsystem);
  }
  public void initialize() {
    armGoal = ArmConstants.restingSetpoint;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(RobotState.getInstance().getState()){
      case AMP:
        armGoal = ArmConstants.ampSetPoint;
        break;
      case CLIMB:
        armGoal = ArmConstants.climbSetpoint;
        break;
      case IDLE:
        armGoal = ArmConstants.restingSetpoint;
        break;
      case INTAKE:
        if(trigger.getAsBoolean()){
          armGoal = ArmConstants.intakeSetpoint;
        }else{
          armGoal = ArmConstants.hoverSetpoint;
        }
        break;
      case SPEAKER:
        //do lookup table here
        break;
    }
    armSubsystem.driveToGoal(armGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
}