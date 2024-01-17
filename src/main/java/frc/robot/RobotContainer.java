// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.ATPositionCmd;
// import frc.robot.commands.ATWAutoCmd;
// import frc.robot.commands.ATWJoystickCmd;
// import frc.robot.commands.ATWPositionCmd;
// import frc.robot.commands.AutoBalanceCommand;
// import frc.robot.commands.ChargingStationAuto;
// import frc.robot.commands.CloseSolenoidCmd;
// import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.commands.DisableCompCmd;
// import frc.robot.commands.DriveXCommand;
// import frc.robot.commands.IntakeDefaultCommand;
// import frc.robot.commands.LockWheelsCmd;
// // import frc.robot.commands.ShootCubeCmd;
// // import frc.robot.commands.PathPlannerCmd;
// import frc.robot.commands.ZeroGyroscope;
// import frc.robot.subsystems.ATWSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.UpdateLimelightCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final ATWSubsystem atwSubsystem = new ATWSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final XboxController m_controller = new XboxController(0);
  // private final Joystick m_js = new Joystick(1);
  // private final Joystick m_js2 = new Joystick(2);
  // private final double[] zeron = {0, 1, -30};//zero
  // private final double[] flatn = {-40, 0.5, -1600};//flat
  // private final double[] zerop = {0, 1, -1800};
  // private final double[] vert = {0, .5, -950};
  //private final double[] forty5 = {-45, .5, -500};//notc
  // private final double[] pickupn = {-117, 7.7, -1300};
  // private final double[] pickupp = {118, 7.7, -700};//pickup
  // private final double[] midn = {-55, 22.7, -725};//mid cone
  // private final double[] midp = {55, 22.7, -1275};
  // private final double[] highn = {-54, 50, -1000};//high cone
  // private final double[] highp = {54, 50, -1000};
  // private final double[] stationp = {44, 11, -1450};
  // private final double[] stationn = {-45, 11, -550};
  // private final double[] hovern = {-90, .5, -160};
  // private final double[] hoverp = {90, .5, -1840};
  // private final double[] siden = {-103, .5, -160};
  // private final double[] sidep = {103, .5, -1840};
  // private final double[] autocubehigh = {-55, 33, -639};
  // private final double[] telecubehigh = {55, 33, -1000};
  // private final SendableChooser<String> autoChooser = new SendableChooser<>();
  // private String m_autoselected = "New Path";
  // private final double pos = -3;
  // private final double pos2 = -2;

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
    // Configure the button bindings
    configureButtonBindings();
    // usbCamera = new UsbCamera("cam", 1);
    // CameraServer.startAutomaticCapture();
   
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheelsCmd(m_drivetrainSubsystem));
    // Back button zeros the gyroscope
    // new Trigger(m_controller::getXButtonPressed).onTrue(new AutoBalanceCommand(
    //     m_drivetrainSubsystem, m_controller::getXButton
    // ) 
    // );
    new Trigger(m_controller::getBackButtonPressed).onTrue(new UpdateLimelightCommand());
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
