// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class test extends SequentialCommandGroup {
  /** Creates a new test. */
  public test(Swerve s_Swerve, XboxController driver) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SpeakerCommand(s_Swerve),new IntakeCommand(s_Swerve));
    /*
     *  public SequentialCommandGroup ampScoringTesting()
  {
    return new SequentialCommandGroup(
      AutoBuilder.pathfindToPose(AutoConstants.AMPP_POSE2D, AutoConstants.pathConstraints),
      new AmpCommand(
      s_Swerve,
      driver,
      () -> operator.getRawButton(1),
      ()->robotControlLeftTrigger.getAsBoolean()
      )
    );
  }
     */
  }
}
