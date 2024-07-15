// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AlienceColorCoordinateFlip;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class codeOrangeFarNote extends SequentialCommandGroup {
  /** Creates a new codeOrangeFarNote. */
  public codeOrangeFarNote(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpeakerCommand(s_Swerve),
      Commands.race(new IntakeBuffer(), AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(156.97))), AutoConstants.pathConstraints)),
      new IntakeCommand(s_Swerve),
      AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(156.97))), AutoConstants.pathConstraints),
      new SpeakerCommand(s_Swerve),
      Commands.race(new IntakeBuffer(), AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(156.97))), AutoConstants.pathConstraints)),
      new IntakeCommand(s_Swerve),
      AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(156.97))), AutoConstants.pathConstraints),
      new SpeakerCommand(s_Swerve)
    );
  }
}
