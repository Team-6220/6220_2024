// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.AutoCmd;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.SpeakerCommand;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PhotonVisionSubsystem;
// import frc.robot.subsystems.Swerve;

// public class AndThenFarNoteCmd extends Command {
//   /** Creates a new AndThenFarNoteCmd. */
//   Swerve s_Swerve;
//   ArmSubsystem arm = ArmSubsystem.getInstance();
//   IntakeSubsystem intake = IntakeSubsystem.getInstance();
//   PhotonVisionSubsystem vis = PhotonVisionSubsystem.getInstance();
//   public AndThenFarNoteCmd(Swerve s_Swerve) {
//     this.s_Swerve = s_Swerve;
//     addRequirements(s_Swerve,arm,intake);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     s_Swerve.setIsAuto(true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     int notePos = AutoConstants.currentCenterNotePos;
//     Command speakerCmd = new SpeakerCommand(s_Swerve);
//     speakerCmd.schedule();
//     .andThen(
//       Commands.race(
//         AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.notePoseIDForAttempting[0]],
//         AutoConstants.pathConstraints), new IntakeBuffer()
//       )
//     )
//     .andThen(vis.getHasTargets() ? new IntakeCommand(s_Swerve)
//       :
//       Commands.race(
//         AutoBuilder.pathfindToPose(AutoConstants.CENTERNOTE_POSE2DS[AutoConstants.notePoseIDForAttempting[1]],
//         AutoConstants.pathConstraints), new IntakeBuffer()
//       )
//       );

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
