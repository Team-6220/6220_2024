package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmTestCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final Joystick js1 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

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

    armSubsystem.setDefaultCommand(
      new ArmTestCommand(
        () -> driver.getAButton(),
        () -> driver.getYButton(),
        () -> driver.getLeftBumper(),
        () -> driver.getRightBumper(),
        () -> js1.getY()
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
