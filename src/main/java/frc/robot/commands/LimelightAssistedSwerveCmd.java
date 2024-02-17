// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class LimelightAssistedSwerveCmd extends Command {
  private final PhotonVisionSubsystem m_VisionSubsystem;
  private final Swerve s_Swerve;
  private final PIDController limelightPidController;
  private final Supplier<Boolean> aButton, autoRange;

  private DoubleSupplier translationSup, strafeSup, rotationSup;

  private final TunableNumber turnkP = new TunableNumber("turnkP", 0);
  private final TunableNumber turnkD = new TunableNumber("turnkD", 0);
  private final TunableNumber turnkI = new TunableNumber("turnkI", 0);
  private final TunableNumber turnTolerance = new TunableNumber("turnTolerance", 3);

  public LimelightAssistedSwerveCmd(Swerve s_Swerve, Supplier<Boolean> aButton, Supplier<Boolean> autoRange, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_VisionSubsystem = PhotonVisionSubsystem.getInstance();
    limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
    this.s_Swerve = s_Swerve;
    this.aButton = aButton;
    this.autoRange = autoRange;
    limelightPidController.setTolerance(turnTolerance.get());
    limelightPidController.setIZone(4);
    
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    addRequirements(m_VisionSubsystem, s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double steerOutput = limelightPidController.calculate(s_Swerve.getHeadingDegrees(),m_VisionSubsystem.getSteeringOffset()) * SwerveConstants.maxAngularVelocity;
    // SmartDashboard.putNumber("BEFORE steeroutput", steerOutput);
    // steerOutput = (steerOutput > SwerveConstants.maxAngularVelocity) ? SwerveConstants.maxAngularVelocity : (steerOutput < -SwerveConstants.maxAngularVelocity) ? -SwerveConstants.maxAngularVelocity : steerOutput;
    // double steerOutput = Normalization(m_VisionSubsystem.getSteeringOffset(), -180, 180, -1, 1) * SwerveConstants.maxAngularVelocity;
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OIConstants.kDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OIConstants.kDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), OIConstants.kDeadband);
    int invert =  (Constants.isRed) ? -1 : 1; 
    double steeroutput = 0;
    if(m_VisionSubsystem.getHasTargets())
    {
      steeroutput = limelightPidController.calculate(m_VisionSubsystem.getTurnOffset());

      steeroutput = (steeroutput > SwerveConstants.maxAngularVelocity)?SwerveConstants.maxAngularVelocity:(steeroutput< -SwerveConstants.maxAngularVelocity)?-SwerveConstants.maxAngularVelocity:steeroutput;
      // System.out.println("success!");
      if(Math.abs(m_VisionSubsystem.getTurnOffset()) < turnTolerance.get()) {
        steeroutput = 0;
      }
      if(autoRange.get())
      {
        strafeVal = 0.1;
      }
    }
    else
    {
      steeroutput = rotationVal * SwerveConstants.maxAngularVelocity;
    }
    SmartDashboard.putNumber("Steeroutput ",steeroutput);
    SmartDashboard.putNumber("Raw Pid output", limelightPidController.calculate(m_VisionSubsystem.getTurnOffset()));
    SmartDashboard.putNumber("Steer_Offset", m_VisionSubsystem.getTurnOffset());
    s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed * invert), 
                steeroutput, 
                true, 
                true
            );

    if(turnkP.hasChanged()
    ||turnkD.hasChanged()
    ||turnkI.hasChanged()) {
      limelightPidController.setPID(turnkP.get(),turnkI.get(),turnkD.get());
    }
    if(turnTolerance.hasChanged()) {
      limelightPidController.setTolerance(turnTolerance.get());
    }
  }

  public double Normalization(double v, double Min, double Max, double newMin, double newMax)
  {
    return (v-Min)/(Max-Min)*(newMax-newMin)+newMin;
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
