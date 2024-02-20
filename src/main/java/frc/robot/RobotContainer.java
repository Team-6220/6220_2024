package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.ArmIdleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeIdleCommand;
import frc.robot.commands.ShooterIdleCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */

    //Trying to add driver control curves
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeTemporary = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton ampTemporary = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton speakerTemporary = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton override = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton noNote = new JoystickButton(driver, XboxController.Button.kStart.value);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_ArmSubsystem = ArmSubsystem.getInstance();
    private final IntakeSubsystem s_IntakeSubsystem = IntakeSubsystem.getInstance();
    private final ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
   //private final PhotonVisionSubsystem p_PhotonVisionSubsystem = PhotonVisionSubsystem.getInstance();


  public RobotContainer() {
    Constants.VisionConstants.setTagHeights();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            driver,
            () -> robotCentric.getAsBoolean()
        )
    );

    
    s_ArmSubsystem.setDefaultCommand(new ArmIdleCommand());
    
    s_ShooterSubsystem.setDefaultCommand(new ShooterIdleCommand());

    s_IntakeSubsystem.setDefaultCommand(new IntakeIdleCommand());
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
    
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(15.3, 5.55), new Rotation2d(0)))));
    
    noNote.onTrue(new InstantCommand(() -> s_IntakeSubsystem.reset()));

    ampTemporary.whileTrue(new AmpCommand(
      s_Swerve,
      driver,
      override)
    );

    intakeTemporary.whileTrue(new IntakeCommand(
      s_Swerve, 
      driver,  
      override).until(() -> s_IntakeSubsystem.noteInTransit())
    );

    speakerTemporary.whileTrue(new SpeakerCommand(
      s_Swerve, 
      driver,
      override)
    );

  }

  public Command getAutonomousCommand() {
    //s_Swerve.setPose(new Pose2d(15.3,5.55,new Rotation2d(0)));
    return autoChooser.getSelected();
  }

}
