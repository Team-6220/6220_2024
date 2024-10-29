// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberTestCommand extends Command {
//   /** Creates a new ClimberTestCommand. */
//   private ClimberSubsystem s_ClimberSubsystem;
//   private final Joystick operator;
//   public ClimberTestCommand(Joystick operator) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     s_ClimberSubsystem = ClimberSubsystem.getInstance();
//     this.operator = operator;
//     addRequirements(s_ClimberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double joystickYValue = -operator.getRawAxis(Joystick.AxisType.kY.value);
//     double joystickTwistValue = operator.getRawAxis(Joystick.AxisType.kZ.value);
//     double outputLeft = 0;
//     double outputRight = 0;
//     boolean drive = false;
//     if(Math.abs(joystickYValue) > .2) {
//       outputLeft = joystickYValue;
//       outputRight = joystickYValue;
//       drive = true;
//     }
//     if(joystickTwistValue < -.2) {
//       outputLeft = joystickTwistValue;
//       drive = true;
//     } else if(joystickTwistValue > .2) {
//       outputRight = -joystickTwistValue;
//       drive = true;
//     }

//     if(drive) {
//       //System.out.println(outputLeft);
//       s_ClimberSubsystem.simpleDriveRight(outputRight);
//       s_ClimberSubsystem.simpleDriveLeft(outputLeft);
//     } else {
//       s_ClimberSubsystem.simpleDriveRight(0);
//       s_ClimberSubsystem.simpleDriveLeft(0);
//     }
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     s_ClimberSubsystem.simpleDriveRight(0);
//     s_ClimberSubsystem.simpleDriveLeft(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
