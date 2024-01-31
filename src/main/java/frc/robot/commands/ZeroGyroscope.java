// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class ZeroGyroscope extends Command{
//     private final DrivetrainSubsystem m_swerveSubsystem;
//     public ZeroGyroscope(DrivetrainSubsystem SwerveSubsystem){
//         this.m_swerveSubsystem = SwerveSubsystem;
//         addRequirements(SwerveSubsystem);
//     }
//     @Override
//     public void initialize(){
//         this.m_swerveSubsystem.zeroHeading();
//         this.m_swerveSubsystem.resetOdometry(m_swerveSubsystem.getPose());
//     }
//     @Override
//     public void execute(){
//         //do nothing
//     }
//     @Override
//     public boolean isFinished(){
//         return true;
//     }
//     @Override
//     public void end(boolean interrupted){
//         //do nothing
//     }
// }
