package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton resetToAbsolute = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton visionAutoAlign = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

  public RobotContainer() {
    Constants.VisionConstants.setTagHeights();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
        )
    );

    configureButtonBindings();
   
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    resetToAbsolute.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    // visionAutoAlign.onTrue(new LimelightAssistedSwerveCmd(s_Swerve));
    new Trigger(driver :: getAButtonPressed).onTrue(new LimelightAssistedSwerveCmd(s_Swerve));
    
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
