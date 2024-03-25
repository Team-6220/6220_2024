// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.AutoCmd;

// import java.util.ArrayList;
// import java.util.Collections;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.EventMarker;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.lib.util.Auto_GetMostRecentPoses;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.SpeakerCommand;
// import frc.robot.subsystems.Swerve;

// public class FarNoteCommand extends Command {
//   /** Creates a new FarNoteCommand. */
//   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(Auto_GetMostRecentPoses.getLatestPosesForBezier());
//   List<EventMarker> eventMarkers = new ArrayList<EventMarker>();
//   Swerve s_Swerve;
//   PathPlannerPath path;
//   public FarNoteCommand(Swerve s_Swerve) {
//     this.s_Swerve = s_Swerve;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(s_Swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     for(int i = AutoConstants.currentCenterNotePos; i < AutoConstants.centerNoteLimit; i ++)
//     {
//       eventMarkers.add(new EventMarker(i, new CheckEmptyCenterNote()));
//     }
//     path = new PathPlannerPath(bezierPoints, Collections.emptyList(), Collections.emptyList(), eventMarkers, AutoConstants.pathConstraints, new GoalEndState(0, AutoConstants.CENTERNOTE_POSE2DS[0].getRotation()), false, AutoConstants.CENTERNOTE_POSE2DS[0].getRotation()); //Change if needed
//     s_Swerve.setIsAuto(true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(AutoConstants.currentCenterNotePos < AutoConstants.centerNoteLimit)
//     {
//       Commands.race(AutoBuilder.pathfindThenFollowPath(path, AutoConstants.pathConstraints, 3.5), new IntakeBuffer());
//       System.out.println("hi");
//       end(true);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//       return false;
//   }
// }
