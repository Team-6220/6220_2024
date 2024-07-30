// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.blinkin;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.lib.util.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final Supplier<Boolean> shootSupplier, autoControl; // Use this so that it's the driver click the button for it to shoot.
  private final ProfiledPIDController leftAndRightPID, fowardAndBackPID; //from the driver's point of view and 0,0 is at the right hand side of the driver
  private blinkin s_Blinkin;
  private boolean isParallelingWithAutobuilder = false;

  private boolean fieldRelative = true;

  //Vision stuff for auto align after path finding
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
  
  /** Creates a new AmpTestCmd. */
  public AmpCommand(Swerve s_Swerve, XboxController driver, Supplier<Boolean> shootSupplier, Supplier<Boolean> autoControl) {
    armSubsystem = ArmSubsystem.getInstance();
    shooterSubsystem = ShooterSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
    s_Blinkin = blinkin.getInstance();
    this.shootSupplier = shootSupplier;
    this.s_Swerve = s_Swerve;
    this.driver = driver;
    this.autoControl = autoControl;
    
    fowardAndBackPID = new ProfiledPIDController(kP.get(), kI.get(),kD.get(), new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    fowardAndBackPID.setTolerance(Tolerance.get());
    
    leftAndRightPID = new ProfiledPIDController(kP.get(), kI.get(), kD.get(),  new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    leftAndRightPID.setTolerance(Tolerance.get());
    
    addRequirements(armSubsystem,shooterSubsystem, s_Swerve, s_Blinkin);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // public AmpCommand(Swerve s_Swerve, Supplier<Boolean> shootSupplier, XboxController driver)
  // {
  //   armSubsystem = ArmSubsystem.getInstance();
  //   shooterSubsystem = ShooterSubsystem.getInstance();
  //   intakeSubsystem = IntakeSubsystem.getInstance();
  //   s_Blinkin = blinkin.getInstance();
  //   this.shootSupplier = shootSupplier;
  //   this.s_Swerve = s_Swerve;
  //   this.driver = driver;
  //   autoControl = () -> true;
  //   System.out.println("hi");
  // }

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
    double xOutput = 0, yOutput = 0, rotationVal = 0;
    if(!isParallelingWithAutobuilder && !autoControl.get()) {
      xOutput = driverInputs[0];
      yOutput = driverInputs[1];
      rotationVal = driverInputs[2];
      
      fieldRelative = true;

      s_Blinkin.solid_purple();
    }
    else {
      // double xOffset, yOffset, validTarget, lastTimeStampSeconds;
      // NetworkTableEntry tx,ty;
      
      // double validTarget = table.getEntry("tv").getDouble(0.0);
      // if(validTarget != 0)
      // {
        // double yOffset = table.getEntry("ty").getDouble(0.0);
        // double xOffset =  table.getEntry("tx").getDouble(0.0);
  
        // double verticalOffset = yOffset; //add anything if needed
        // double angle = (verticalOffset + VisionConstants.limelightAngleDegrees) * (3.14159/180);
        // double yDistanceFromLimelightToGoalInches = (53.38 - VisionConstants.heightOfCamAboveFloor)/Math.tan(angle);

        // double yDistanceKp = 0.001;
        // double distance_error_y = yDistanceKp *(VisionConstants.desiredDistanceToAprilTagY - yDistanceFromLimelightToGoalInches);
        // xOutput = distance_error_y;

        // double horizontalOffset = xOffset; //add anything if needed
        // double angle_x = (horizontalOffset + VisionConstants.limelightAngleDegrees) * (3.14159/180);
        // double xDistanceFromLimelightToGoalInches = (53.38 - VisionConstants.heightOfCamAboveFloor)/Math.tan(angle);
        // double xDistanceKp = 0.001;
        // double distance_error_x = xDistanceKp *(VisionConstants.desiredDistanceToAprilTagY - yDistanceFromLimelightToGoalInches);
        // yOutput = distance_error_x;
        // boolean fieldRelative = false;
      // }
      // else{
      //   System.err.println("APRIL TAG NOT DETECTED");
      // }
      // fowardAndBackPID.setGoal(s_Swerve.getAmpX());
      // leftAndRightPID.setGoal(s_Swerve.getAmpY());
      
      SmartDashboard.putNumber("heading swerve", s_Swerve.getHeadingDegrees());
      // SmartDashboard.putNumber("x setpoint", fowardAndBackPID.getSetpoint().position);
      // SmartDashboard.putNumber("y setpoint", leftAndRightPID.getSetpoint().position);
      
      // xOutput = fowardAndBackPID.calculate(s_Swerve.getPose().getX());
      // yOutput = leftAndRightPID.calculate(s_Swerve.getPose().getY());
      
      // s_Swerve.setAutoTurnHeading(90);
      // rotationVal = s_Swerve.getTurnPidSpeed();
      
      s_Blinkin.sky_blue();
    }
    if(!isParallelingWithAutobuilder)
    {
      s_Swerve.drive(
      new Translation2d(xOutput, yOutput),
      rotationVal,
      fieldRelative,
      true
      );
    }
    // if(Math.hypot(s_Swerve.getPose().getX() - s_Swerve.getAmpX(), s_Swerve.getPose().getY()-s_Swerve.getAmpY()) < 1.75) {
    // } else {
    //   armSubsystem.driveToGoal(ArmConstants.restingSetpoint);
    // }
    armSubsystem.driveToGoal(ArmConstants.ampSetPoint);

    
    if(shootSupplier.get()) {
      shooterSubsystem.fireAmp();
      intakeSubsystem.feedAmp();
      s_Blinkin.solid_green();
    } else {
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
