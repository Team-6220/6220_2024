// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);

  /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_controller, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // autoChooser.setDefaultOption("New Path", "New Path");
    // autoChooser.addOption("MidMobA", "MidMobA");
    // autoChooser.addOption("MidMobB", "MidMobB");
    // autoChooser.addOption("MidClimb", "MidClimb");
    // autoChooser.addOption("MidClimbMob", "MidClimbMob");
    // autoChooser.addOption("Top","Top");
    // autoChooser.addOption("Bot", "Bot");
    // autoChooser.addOption("BotClimb", "BotClimb");
    // autoChooser.addOption("FollowBot", "FollowBot");
    // SmartDashboard.putData(autoChooser);
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5
    // ));
//     atwSubsystem.setDefaultCommand(new ATWJoystickCmd(
//       atwSubsystem,
//       () -> 0d,
//       () -> -m_js.getY(),
//       () -> 0d

// ));
// intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(
//   intakeSubsystem,
//   m_js::getTriggerPressed,
//   () -> m_js.getRawButtonPressed(2), 
//   m_js2::getTrigger,
//   () -> m_js2.getRawButton(2)
// ));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -m_controller.getRawAxis(translationAxis), 
                () -> -m_controller.getRawAxis(strafeAxis), 
                () -> -m_controller.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
    // Configure the button bindings
    configureButtonBindings();
   
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    //new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheelsCmd(m_drivetrainSubsystem));
    // Back button zeros the gyroscope
    // new Trigger(m_controller::getXButtonPressed).onTrue(new AutoBalanceCommand(
    //     m_drivetrainSubsystem, m_controller::getXButton
    // ) 
    // );
        //new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        //new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));
        //zero that bih!
        // new Trigger(() -> m_js2.getRawButtonPressed(7)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     zeron,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButtonPressed(8)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     flatn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(7)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     zerop,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //flat like a door (or your mother)
        // new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     flatn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //the thing that a notc would do
        // new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     forty5,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //pick that shi up
        // new Trigger(() -> m_js2.getRawButtonPressed(3)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     pickupn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //mid cone (negative direction)
        // new Trigger(() -> m_js2.getRawButtonPressed(4)).onTrue(
        //     new ATWPositionCmd(atwSubsystem,
        //      autocubehigh,
        //      () -> m_js.getThrottle(),
        //      () -> m_js2.getThrottle())
        // );
        
        //high cone (negative direction)
        // new Trigger(() -> m_js2.getRawButtonPressed(5)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     highn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     pickupp,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(4)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     telecubehigh,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(5)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     highp,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButtonPressed(6)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     stationn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(6)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     stationp,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     hoverp,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     sidep,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     hovern,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     siden,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButton(10)).onTrue(new ATPositionCmd(
        //   atwSubsystem, zeron, () -> m_js.getThrottle(),
        //   () -> m_js2.getThrottle())
        // );
        // new Trigger(() -> m_js2.getRawButtonPressed(9)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     midn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(9)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     midp,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new SequentialCommandGroup(
    //   new ZeroGyroscope(m_drivetrainSubsystem),
    //   new DriveXCommand(m_drivetrainSubsystem, pos)
    // );
    // m_autoselected = autoChooser.getSelected();
    return new SequentialCommandGroup(
      // new DisableCompCmd(intakeSubsystem),
      // //new CloseSolenoidCmd(intakeSubsystem),//for cone
      // new ATWAutoCmd(atwSubsystem, autocubehigh, null, null),
      // //new ATWAutoCmd(atwSubsystem, highn, null, null),
      // new ShootCubeCmd(intakeSubsystem),
      // //new CloseSolenoidCmd(intakeSubsystem),//for cone only
      // new ATWAutoCmd(atwSubsystem, zeron, null, null),
      // new CloseSolenoidCmd(intakeSubsystem),
      // new PathPlannerCmd(m_drivetrainSubsystem, atwSubsystem, intakeSubsystem, "OneMeter"),
      // new ChargingStationAuto(m_drivetrainSubsystem),
      //new LockWheelsCmd(m_drivetrainSubsystem)
    );

    
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.06);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
