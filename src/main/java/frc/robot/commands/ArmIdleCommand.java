// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.blinkin;


// public class ArmIdleCommand extends Command {
//   /** Creates a new IdleCommand. */
//   ArmSubsystem s_ArmSubsystem;
//   blinkin s_Blinkin;

//   public ArmIdleCommand() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     s_ArmSubsystem = ArmSubsystem.getInstance();
//     s_Blinkin = blinkin.getInstance();
//     addRequirements(s_ArmSubsystem, s_Blinkin);
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     s_ArmSubsystem.driveToGoal(ArmConstants.restingSetpoint);
//     s_Blinkin.solid_gold();
//   }
//   @Override
//   public void end(boolean interrupted) {
//     s_ArmSubsystem.stop();
//   } 

// }
