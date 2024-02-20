// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AmpCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final XboxController driver;
  private final Swerve s_Swerve;
  private final JoystickButton shoot; // Use this so that it's the driver click the button for it to shoot.

  /** Creates a new AmpTestCmd. */
  public AmpCommand(Swerve s_Swerve, XboxController driver, JoystickButton shoot) {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
    this.shoot = shoot;
    this.s_Swerve = s_Swerve;
    this.driver = driver;
    addRequirements(armSubsystem,shooterSubsystem, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] driverInputs = OIConstants.getDriverInputs(driver);

    s_Swerve.setAutoTurnHeading(90);
    double rotationVal = s_Swerve.getTurnPidSpeed();
    s_Swerve.drive(
      new Translation2d(driverInputs[0], driverInputs[1]),
      rotationVal,
      true,
      true
    );

    armSubsystem.driveToGoal(armSubsystem.armAmpAngle.get());
    
      if(shoot.getAsBoolean())
      {
        shooterSubsystem.spinManually(ArmConstants.ampShooterSpeed);
        intakeSubsystem.feedAmp();
      } else {
        shooterSubsystem.stop();
        intakeSubsystem.stop();
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    shooterSubsystem.stop();
    intakeSubsystem.stop();
  }

  
}
