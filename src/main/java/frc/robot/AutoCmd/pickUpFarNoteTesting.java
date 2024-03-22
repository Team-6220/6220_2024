// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Auto_GetMostRecentPoses;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeFarNoteCmd;
import frc.robot.commands.SpeakerCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pickUpFarNoteTesting extends SequentialCommandGroup {
  /** Creates a new pickUpFarNoteTesting. */
  // List<RotationTarget> holonomicRotations = new ArrayList<RotationTarget>();
  public pickUpFarNoteTesting(Swerve s_Swerve) {
      // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new IntakeFarNoteCmd(s_Swerve, s_Swerve.getClosestNotePosition())
      new FarNoteCommand(s_Swerve),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      new FarNoteCommand(s_Swerve),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      new FarNoteCommand(s_Swerve),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      new FarNoteCommand(s_Swerve),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve),
      new FarNoteCommand(s_Swerve),
      new IntakeCommand(s_Swerve),
      new SpeakerCommand(s_Swerve)
    );
  }
}
