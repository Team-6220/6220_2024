// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingTest extends Command {

  private final Supplier<Boolean> bButton;

  ArmSubsystem armSubsystem;
  ShooterSubsystem shooterSubsystem;
  /** Creates a new ShootingTest. */

  public ShootingTest(Supplier<Boolean> bB) {
    
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    addRequirements(armSubsystem, shooterSubsystem);

    bButton = bB;
  }

  @Override
    public void execute() {
      armSubsystem.driveToGoal();
      
    }
}
