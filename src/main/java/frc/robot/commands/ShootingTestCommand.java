// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ShooterConfiguration;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class ShootingTestCommand extends Command {

  private final double kP = 175;
  private final double kI = 0;
  private final double kD = 0;
  private final double Vel =  2;
  private final double Accel = 1;
  private final double Tolerance = 0.1;

  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Swerve s_Swerve;
  private XboxController driverInputs;
  private boolean hasFired;
  /** Creates a new ShootingTest. */

  private final ProfiledPIDController leftAndRightPID, fowardAndBackPID;

  private TunableNumber currentRow = new TunableNumber("Current Row", 0);
  private TunableNumber currentColumn = new TunableNumber("Current Column", 0);
  private TunableNumber headingOffsetTest = new TunableNumber("Heading Offset Test", 0);
  public ShootingTestCommand(Swerve s_Swerve, XboxController driverInputs) {
    this.s_Swerve = s_Swerve;
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
    this.driverInputs = driverInputs;
    addRequirements(armSubsystem, shooterSubsystem);

    fowardAndBackPID = new ProfiledPIDController(kP, kI,kD, new TrapezoidProfile.Constraints(Vel, Accel));
    fowardAndBackPID.setTolerance(Tolerance);

    leftAndRightPID = new ProfiledPIDController(kP, kI,kD, new TrapezoidProfile.Constraints(Vel, Accel));
    leftAndRightPID.setTolerance(Tolerance);
  }

  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    fowardAndBackPID.reset(s_Swerve.getPose().getX());
    leftAndRightPID.reset(s_Swerve.getPose().getY());
    hasFired = false;
  }
  @Override
  public void execute() {

    //Pair<Double, Double> setPoint = ShooterConfiguration.polarToCartesian(ShooterConfiguration.radiusValues.get(currentRow.get()), (int)currentColumn.get());


    double xOutput, yOutput, rotationVal;

    //fowardAndBackPID.setGoal(setPoint.getFirst());
    //leftAndRightPID.setGoal(setPoint.getSecond());
    SmartDashboard.putNumber("heading swerve", s_Swerve.getHeadingDegrees());
    SmartDashboard.putNumber("x setpoint", fowardAndBackPID.getSetpoint().position);
    SmartDashboard.putNumber("y setpoint", leftAndRightPID.getSetpoint().position);

    

    xOutput = fowardAndBackPID.calculate(s_Swerve.getPose().getX());
    yOutput = leftAndRightPID.calculate(s_Swerve.getPose().getY());
    //if(Math.abs(s_Swerve.getPose().getX() - setPoint.getFirst()) < .1 && Math.abs(s_Swerve.getPose().getY() - setPoint.getSecond()) < .1) {
    //  xOutput = 0;
    //  yOutput = 0;
    //}

    s_Swerve.setAutoTurnHeading(s_Swerve.getHeadingToSpeaker() + headingOffsetTest.get());
    rotationVal = s_Swerve.getTurnPidSpeed();
  

  
    // s_Swerve.drive(
    //   new Translation2d(xOutput, yOutput),
    // rotationVal,
    // true,
    // true
    // );

    if(driverInputs.getBButton()) {
      armSubsystem.driveToGoal(armSubsystem.armTestAngle.get());
      double[] velocities = {shooterSubsystem.shooterTestVelocityA.get(), shooterSubsystem.shooterTestVelocityB.get()};
      shooterSubsystem.spinToVelocity(velocities);
      if((armSubsystem.isAtGoal() && shooterSubsystem.isAtSetpoint() )|| hasFired){
        intakeSubsystem.feedShooter();
        hasFired = true;
      }
    } else {
      shooterSubsystem.stop();
    }
    if(driverInputs.getAButton()) {
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
