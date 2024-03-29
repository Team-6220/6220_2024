// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPoint;
// import com.pathplanner.lib.path.RotationTarget;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.util.AlienceColorCoordinateFlip;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PhotonVisionSubsystem;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.blinkin;

// public class IntakeFarNoteCmd extends Command {
//   /** Creates a new IntakeFarNoteCmd. */
//   // IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
//   // blinkin LEDs = blinkin.getInstance();
//   // Swerve s_Swerve;
//   // ArmSubsystem arm = ArmSubsystem.getInstance();
//   // PhotonVisionSubsystem vis = PhotonVisionSubsystem.getInstance();
//   int numOfNote;
//   List<PathPoint> pathPoints = new ArrayList<PathPoint>();
//   public IntakeFarNoteCmd(Swerve s_Swerve, int numOfNote) {
//       for(int i = AutoConstants.currentCenterNotePos; i < AutoConstants.CENTERNOTE_POSE2DS.length; i ++)
//       {
//         pathPoints.add(new PathPoint(AutoConstants.CENTERNOTE_POSE2DS[i].getTranslation(), new RotationTarget(i, AutoConstants.CENTERNOTE_POSE2DS[i].getRotation())));
//       }
//     // this.s_Swerve = s_Swinerve;
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.numOfNote = numOfNote;
//     // addRequirements(intakeSubsystem, s_Swerve, arm, vis);
//   }

//   @Override
//   public void execute() {
    
//     // Pose2d nearestFarNote = s_Swerve.getPose().getY() 
//     // TODO: get closest note & add pose 2d into constants
//     PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, AutoConstants.pathConstraints, new GoalEndState(0, AutoConstants.CENTERNOTE_POSE2DS[0].getRotation()));
//     // AutoBuilder.pathfindToPose(
//     //   AutoConstants.CENTERNOTE_POSE2DS[numOfNote],
//     //   AutoConstants.pathConstraints
//     //   );
//     AutoBuilder.pathfindThenFollowPath(path, AutoConstants.pathConstraints);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

// // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(numOfNote > 3)
//     {
//       return true;
//     }
//     return false;
//   }
// }
