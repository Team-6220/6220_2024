// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
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
//   IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
//   blinkin LEDs = blinkin.getInstance();
//   Swerve s_Swerve;
//   ArmSubsystem arm = ArmSubsystem.getInstance();
//   PhotonVisionSubsystem vis = PhotonVisionSubsystem.getInstance();
//   int numOfNote;
//   public IntakeFarNoteCmd(Swerve s_Swerve, int numOfNote) {
//     this.s_Swerve = s_Swerve;
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.numOfNote = numOfNote;
//     addRequirements(intakeSubsystem, s_Swerve, arm, vis);
//   }

//   @Override
//   public void execute() {
    
//     // Pose2d nearestFarNote = s_Swerve.getPose().getY() 
//     // TODO: get closest note & add pose 2d into constants
//     AutoBuilder.pathfindToPose(
//       AutoConstants.CENTERNOTE_POSE2DS[numOfNote],
//       AutoConstants.pathConstraints
//       );
//     arm.driveToGoal(ArmConstants.intakeSetpoint);
//     if(!vis.getHasTargets())
//     {
//       new IntakeFarNoteCmd(s_Swerve, numOfNote++);
//     }
//     else if(vis.getHasTargets())
//     {
//       new IntakeCommand(s_Swerve);
//     }
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
//     if(numOfNote  -1)
//     {
//       return true;
//     }
//     return false;
//   }
// }
