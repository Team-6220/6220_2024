package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */

    //Trying to add driver control curves
    

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

  public RobotContainer() {
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

    configureButtonBindings();
   
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
