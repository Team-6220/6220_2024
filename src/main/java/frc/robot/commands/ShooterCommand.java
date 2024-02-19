package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double[] velocities;

    public ShooterCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        velocities = new double[2];
        velocities[0] = 0;
        velocities[1] = 0;
    }

    @Override
    public void execute(){
        switch(RobotState.getInstance().getState()){
            case AMP:
                //get amp speed
                break;
            case SPEAKER:
                //get shooter speed
                break;
            default:
                velocities[0] = 0;
                velocities[1] = 0;
                break;
        }
        shooterSubsystem.spinToVelocity(velocities);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stop();
    }
}
