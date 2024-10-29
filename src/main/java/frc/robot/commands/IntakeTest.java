// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

// public class IntakeTest extends Command {
//   IntakeSubsystem s_intakeSubsystem;
//   ArmSubsystem s_armSubsystem;
//   /** Creates a new IntakeTest. */
//   public IntakeTest() {
//     s_intakeSubsystem = IntakeSubsystem.getInstance();
//     s_armSubsystem = ArmSubsystem.getInstance();
//     addRequirements(s_intakeSubsystem, s_armSubsystem);
//     //addRequirements(s_armSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     s_intakeSubsystem.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double armPos = 0;
//     if(s_intakeSubsystem.getNoteInIntake()) {
//       armPos = 45;
//     } else {
//       armPos = ArmConstants.intakeSetpoint;
//     }
//     s_armSubsystem.driveToGoal(armPos);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     s_armSubsystem.stop();
//     s_intakeSubsystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
