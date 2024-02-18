package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.commands.*;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final Joystick operator = new Joystick(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton aimToHeading = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton aimToSpeaker = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton aimToNote = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  /* Operator Buttons */
  private final JoystickButton idleMode = new JoystickButton(operator, 7);
  private final JoystickButton intakeMode = new JoystickButton(operator, 8);
  private final JoystickButton ampMode = new JoystickButton(operator, 9);
  private final JoystickButton speakerMode = new JoystickButton(operator, 10);
  private final JoystickButton climbMode = new JoystickButton(operator, 11);
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  //private final PhotonVisionSubsystem p_PhotonVisionSubsystem = PhotonVisionSubsystem.getInstance();

  public enum RobotState{
    IDLE,
    INTAKE,
    AMP,
    SPEAKER,
    CLIMB
  }

  public RobotState robotState; 

  public RobotContainer() {
    this.robotState = RobotState.IDLE;

    Constants.VisionConstants.setTagHeights();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
            () -> OIConstants.modifyRotAxis(-driver.getRawAxis(rotationAxis)), 
            () -> robotCentric.getAsBoolean()
        )
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(15.3, 5.55), new Rotation2d(0)))));
    aimToSpeaker.whileTrue(new TeleopAimSwerve(
        s_Swerve,
        () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
        () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
        () -> s_Swerve.getHeadingToSpeaker()
      )
    );
    aimToHeading.whileTrue(
      new TeleopAimSwerve(
        s_Swerve,
        () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
        () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
        () -> -90
      )
    );
    aimToNote.whileTrue(new ShootingTestCommand());
    intake.whileTrue(new IntakeTest());
    idleMode.onTrue(new InstantCommand(() -> robotState = RobotState.IDLE));
    intakeMode.onTrue(new InstantCommand(() -> robotState = RobotState.INTAKE));
    ampMode.onTrue(new InstantCommand(() -> robotState = RobotState.AMP));
    speakerMode.onTrue(new InstantCommand(() -> robotState = RobotState.SPEAKER));
    climbMode.onTrue(new InstantCommand(() -> robotState = RobotState.CLIMB));
  }

  public Command getAutonomousCommand() {
    //s_Swerve.setPose(new Pose2d(15.3,5.55,new Rotation2d(0)));
    return autoChooser.getSelected();
  }

}
