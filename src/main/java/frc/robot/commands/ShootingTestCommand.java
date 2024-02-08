// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.lib.util.TunableNumber;

public class ShootingTestCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  /** Creates a new ShootingTest. */

  public ShootingTestCommand() {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();

    addRequirements(armSubsystem, shooterSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.driveToGoal(armSubsystem.armTestAngle.get());
    shooterSubsystem.spinToVelocity(shooterSubsystem.shooterTestVelocity.get());
    //System.out.println("Shooter Test NUm: " + shooterSubsystem.shooterTestVelocity.get());
    if(armSubsystem.isAtGoal() && shooterSubsystem.isAtSetpoint()){
      //System.out.println("Fire");
      intakeSubsystem.feedShooter();
    }
  }

  @Override
  public void end(boolean interrupted){
    armSubsystem.stop();
    shooterSubsystem.stop();
    intakeSubsystem.stop();
  }
}
