package frc.robot.commands;

import frc.robot.subsystems.Swerve;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;


public class TurnToHeading extends Command {    
    private Swerve s_Swerve;    
    private double heading;
    public TurnToHeading(Swerve s_Swerve, double heading) {
        this.s_Swerve = s_Swerve;
        this.heading = heading;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
        s_Swerve.setIsAutoTurning(true);
        s_Swerve.setTurnControllerGoal(heading);
    }

    @Override
    public void execute() {
        s_Swerve.setAutoTurnHeading(heading);
    }

    public void setHeading(double heading) {
        this.heading = heading;
        s_Swerve.setIsAutoTurning(true);
    }

    public boolean isFacingHeading() {
        if(s_Swerve.isFacingTurnTarget()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setIsAutoTurning(false);
    }

    @Override
    public boolean isFinished(){
        return isFacingHeading();
    }
}