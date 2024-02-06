// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingTestCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  TunableNumber armAngle = new TunableNumber("Shooter Test Arm Setpoint", 0);
  TunableNumber shooterVelocity = new TunableNumber("Shooter Test Velocity", 0);

  /** Creates a new ShootingTest. */

  public ShootingTestCommand() {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();

    addRequirements(armSubsystem, shooterSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.driveToGoal(armAngle.get());
    shooterSubsystem.spinToVelocity(shooterVelocity.get());
    if(Math.abs(armSubsystem.getArmPosition() - armAngle.get()) <= 2 && Math.abs(shooterSubsystem.getVelocity() - shooterVelocity.get()) <= 10){
      intakeSubsystem.simpleDrive(false);
    }
  }

  @Override
  public void end(boolean interrupted){
    armSubsystem.stop();
    shooterSubsystem.stop();
    intakeSubsystem.stop();
  }
}
