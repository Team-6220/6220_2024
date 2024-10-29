// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ArmSubsystem;

// public class Tuning_Arm extends Command {
//   /** Creates a new Tuning_Arm. */
//   private final ArmSubsystem s_ArmSubsystem;
//   private final XboxController driver;
//   public Tuning_Arm(XboxController driver) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.driver = driver;
//     s_ArmSubsystem = ArmSubsystem.getInstance();
//     addRequirements(s_ArmSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(driver.getXButton()) {
//       s_ArmSubsystem.driveToGoal(s_ArmSubsystem.armTestAngle.get());
//     } else if(driver.getLeftBumper()) {
//       s_ArmSubsystem.simpleDrive(.1);
//     } else if(driver.getRightBumper()) {
//       s_ArmSubsystem.simpleDrive(-.1);
//     } else {
//       s_ArmSubsystem.stop();
//     }
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
