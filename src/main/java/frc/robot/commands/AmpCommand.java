// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import frc.lib.util.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class AmpCommand extends Command {

  private final TunableNumber kP = new TunableNumber(" ampkP", 175);
  private final TunableNumber kI = new TunableNumber(" ampkI", 0);
  private final TunableNumber kD = new TunableNumber(" ampkD", 0);
  private final TunableNumber Vel = new TunableNumber(" MaxVel", 3);
  private final TunableNumber Accel = new TunableNumber(" Accel", 2);
  private final TunableNumber Tolerance = new TunableNumber(" Tolerance", 0.01);

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final XboxController driver;
  private final Swerve s_Swerve;
  private final Supplier<Boolean> shootSupplier, manuelOverride; // Use this so that it's the driver click the button for it to shoot.
  private final ProfiledPIDController leftAndRightPID, fowardAndBackPID; //from the driver's point of view and 0,0 is at the right hand side of the driver

  /** Creates a new AmpTestCmd. */
  public AmpCommand(Swerve s_Swerve, XboxController driver, Supplier<Boolean> shootSupplier, Supplier<Boolean> override) {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
    this.shootSupplier = shootSupplier;
    this.s_Swerve = s_Swerve;
    this.driver = driver;
    manuelOverride = override;

    fowardAndBackPID = new ProfiledPIDController(kP.get(), kI.get(),kD.get(), new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    fowardAndBackPID.setTolerance(Tolerance.get());

    leftAndRightPID = new ProfiledPIDController(kP.get(), kI.get(), kD.get(),  new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    leftAndRightPID.setTolerance(Tolerance.get());

    addRequirements(armSubsystem,shooterSubsystem, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    fowardAndBackPID.reset(s_Swerve.getPose().getX());
    leftAndRightPID.reset(s_Swerve.getPose().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] driverInputs = OIConstants.getDriverInputs(driver);
    double xOutput, yOutput, rotationVal;
    if(manuelOverride.get())
    {
      xOutput = driverInputs[0];
      yOutput = driverInputs[1];
      rotationVal = driverInputs[2];
    }
    else
    {
      fowardAndBackPID.setGoal(s_Swerve.getForwardBackwardToAmp());
      leftAndRightPID.setGoal(s_Swerve.getLeftAndRightToAmp());
      SmartDashboard.putNumber("heading swerve", s_Swerve.getHeadingDegrees());
      SmartDashboard.putNumber("x setpoint", fowardAndBackPID.getSetpoint().position);
      SmartDashboard.putNumber("y setpoint", leftAndRightPID.getSetpoint().position);
      xOutput = fowardAndBackPID.calculate(s_Swerve.getPose().getX());
      yOutput = leftAndRightPID.calculate(s_Swerve.getPose().getY());
      s_Swerve.setAutoTurnHeading(90);
      rotationVal = s_Swerve.getTurnPidSpeed();
    }

    
      s_Swerve.drive(
        new Translation2d(xOutput, yOutput),
      rotationVal,
      true,
      true
    );

    armSubsystem.driveToGoal(armSubsystem.armAmpAngle.get());
    
      if(fowardAndBackPID.atGoal() && leftAndRightPID.atGoal() && armSubsystem.isAtGoal())
      {
        shooterSubsystem.spinManually(ArmConstants.ampShooterSpeed);
        intakeSubsystem.feedAmp();
      }
      else if(shootSupplier.get())
      {
        shooterSubsystem.spinManually(ArmConstants.ampShooterSpeed);
        intakeSubsystem.feedAmp();
      }
      else {
        shooterSubsystem.stop();
        intakeSubsystem.stop();
      }

      if(kP.hasChanged()
      || kI.hasChanged()
      || kD.hasChanged()) {
          fowardAndBackPID.setPID(kP.get(), kI.get(), kD.get());
          leftAndRightPID.setPID(kP.get(), kI.get(), kD.get());
          leftAndRightPID.reset(s_Swerve.getPose().getY());
          fowardAndBackPID.reset(s_Swerve.getPose().getX());
      }
      if(Vel.hasChanged() || Accel.hasChanged()) {
          fowardAndBackPID.setConstraints(new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
          leftAndRightPID.setConstraints(new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
          leftAndRightPID.reset(s_Swerve.getPose().getY());
          fowardAndBackPID.reset(s_Swerve.getPose().getX());
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
