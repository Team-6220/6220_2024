// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AmpTestCmd extends Command {

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final Supplier<Boolean> shootSupplier; // Use this so that it's the driver click the button for it to shoot.

  /** Creates a new AmpTestCmd. */
  public AmpTestCmd(Supplier<Boolean> shootSupplier) {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
    this.shootSupplier = shootSupplier;
    addRequirements(armSubsystem,shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.driveToGoal(armSubsystem.armAmpAngle.get());
    if(armSubsystem.isAtGoal())
    {
      if(shootSupplier.get())
      {
        shooterSubsystem.spinManually(ArmConstants.ampShooterSpeed);
        intakeSubsystem.feedAmp();
      }
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
