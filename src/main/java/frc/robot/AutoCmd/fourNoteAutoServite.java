// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AlienceColorCoordinateFlip;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fourNoteAutoServite extends SequentialCommandGroup {
  /** Creates a new fourNoteAutoServite. */
  ArmSubsystem s_ArmSubsystem = ArmSubsystem.getInstance();
  public fourNoteAutoServite(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new SpeakerCommand(s_Swerve),
      // Commands.deadline(AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-156.97))), AutoConstants.pathConstraints), new IntakeCommand(s_Swerve, true)),
      Commands.race(new IntakeBuffer(), AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(1.80),4.35, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-156.97))), AutoConstants.pathConstraints)),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      // Commands.deadline(AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.85), 4.85, Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(-90))), AutoConstants.pathConstraints), new IntakeCommand(s_Swerve, true)),
      Commands.race(new IntakeBuffer(), AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.85), 4.85, Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(-90))), AutoConstants.pathConstraints)),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      // Commands.deadline(AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.9), 6.3, Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(-90))), AutoConstants.pathConstraints), new IntakeCommand(s_Swerve, true)),
      Commands.race(new IntakeBuffer(), AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.85), 6.3, Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(-90))), AutoConstants.pathConstraints)),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve)
    );
    // ListOfAllAutos.addNewAuto("four note auto servite", this);
    // AutoBuilder.buildAuto(fourNoteAutoServite(s_Swerve));
  }
}
