// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;

// import java.util.function.Supplier;
// import frc.robot.subsystems.ArmSubsystem;

// public class ArmManuelCmd extends Command {
//   /** Creates a new ArmManuelCmd. */
//   private final ArmSubsystem armSubsystem;
//   private final Supplier<Double> armInput;
//   public ArmManuelCmd(Supplier<Double> armInput) {
//     // Use addRequirements() here to declare subsystem dependencies.
//   armSubsystem = ArmSubsystem.getInstance();
//   this.armInput = armInput;
//   addRequirements(armSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     if (Math.abs(armInput.get()) > 0.1)
//         {
//           armSubsystem.setArmMotorManual(armInput.get());
//           System.out.println("success");
//         }
//         else
//         {
//             armSubsystem.setArmMotorManual(0);
//         }
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
