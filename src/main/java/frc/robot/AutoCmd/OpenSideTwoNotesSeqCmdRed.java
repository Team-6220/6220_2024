// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.AutoCmd;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

// // import java.util.ArrayList;
// // import java.util.Collections;
// // import java.util.List;

// // import com.pathplanner.lib.auto.AutoBuilder;
// // import com.pathplanner.lib.path.EventMarker;
// // import com.pathplanner.lib.path.GoalEndState;
// // import com.pathplanner.lib.path.PathPlannerPath;
// // import com.pathplanner.lib.path.PathPoint;
// // import com.pathplanner.lib.path.RotationTarget;

// // import edu.wpi.first.math.Pair;
// // import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.geometry.Translation2d;
// // import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.lib.util.AlienceColorCoordinateFlip;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.AutoSystemsPrep;
// // import frc.lib.util.Auto_GetMostRecentPoses;
// // import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.IntakeCommand;
// // import frc.robot.commands.IntakeFarNoteCmd;
// import frc.robot.commands.SpeakerCommand;
// import frc.robot.subsystems.Swerve;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class OpenSideTwoNotesSeqCmdRed extends SequentialCommandGroup {
//   /** Creates a new pickUpFarNoteTesting. */
//   // List<RotationTarget> holonomicRotations = new ArrayList<RotationTarget>();
//   private double bufferVelocityForInBetweenPaths = AutoConstants.bufferVelocityForInBetweenPaths; //default globle, can chagne if needed
//   private double bufferVelocityForShoot = AutoConstants.bufferVelocityForShooting;
//   private double bufferVelocityForIntake = AutoConstants.bufferVelocityForIntake;
//   // private Pose2d firePoint = new Pose2d(AlienceColorCoordinateFlip.flip(1.85), 3.5, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(140)).getRadians()));
//   public OpenSideTwoNotesSeqCmdRed(Swerve s_Swerve) {
//       // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       // new InstantCommand(() -> s_Swerve.setPose(new Pose2d(AlienceColorCoordinateFlip.flip(0.85), 4.40, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(120)).getRadians())))),
//       new SpeakerCommand(s_Swerve),
//       //AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.90), 6.25, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(180)).getRadians())), AutoConstants.pathConstraints, 2),
//       //AutoBuilder.pathfindToPose(new Pose2d(AlienceColorCoordinateFlip.flip(2.90), 6.25, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(180)).getRadians())), AutoConstants.pathConstraints, 2),
//       AutoBuilder.pathfindToPose(AutoConstants.MULTITARGETPOSES_FORINTAKECAMERA_RED[2], AutoConstants.pathConstraints, bufferVelocityForIntake, 1),
//       new IntakeCommand(s_Swerve),
//       Commands.deadline(AutoBuilder.pathfindToPose(AutoConstants.openSideShootingPose, AutoConstants.pathConstraints, bufferVelocityForShoot), new AutoSystemsPrep(AutoConstants.openSideShootingPose, s_Swerve)),
//       new SpeakerCommand(s_Swerve),
//       AutoBuilder.pathfindToPose(AutoConstants.MULTITARGETPOSES_FORINTAKECAMERA_RED[2], AutoConstants.pathConstraints, bufferVelocityForIntake, 1),
//       // Commands.waitSeconds(.1),
//       new IntakeCommand(s_Swerve),
//       Commands.deadline(AutoBuilder.pathfindToPose(AutoConstants.openSideShootingPose, AutoConstants.pathConstraints, bufferVelocityForShoot), new AutoSystemsPrep(AutoConstants.openSideShootingPose, s_Swerve)),
//       new SpeakerCommand(s_Swerve)
//     );
//   }

//   /**
//    * @deprecated because too buggy and intake doesn't run apropriately all the time, chagned instead to make intake decide.
//    */
//   // private Command isThereNote(Swerve s_Swerve) {
//   //   //s_VisionSubsystem.getHasTargets()
//   //   if(s_VisionSubsystem.getHasTargets()) {
//   //     AutoConstants.currentCenterNotePos ++;
//   //     System.out.println("Auto Sees note");
//   //     return new IntakeCommand(s_Swerve);
//   //   } else {
//   //           System.out.println("Auto does not see note");
//   //     AutoConstants.currentCenterNotePos ++;
//   //     if(AutoConstants.currentCenterNotePos < AutoConstants.howManyNotesAreWeAttempting){
//   //       return new SequentialCommandGroup(AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.notePoseIDForAttempting[AutoConstants.currentCenterNotePos]], AutoConstants.pathConstraints, 0, 1.5),Commands.waitSeconds(.4), isThereNote(s_Swerve));
//   //     }
//   //     else
//   //     {
//   //       AutoConstants.currentCenterNotePos --;
//   //       return AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.notePoseIDForAttempting[AutoConstants.currentCenterNotePos]], AutoConstants.pathConstraints);
//   //     }
//   //   }
    
//   // }
// }
