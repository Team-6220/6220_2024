package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers;

public class AimToSpeaker extends Command {
    private VisionSubsystem s_VisionSubsystem;
    private boolean hasSeenTarget;
    private Swerve s_Swerve;    

    public AimToSpeaker(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        s_VisionSubsystem = VisionSubsystem.getInstance();
        hasSeenTarget = false;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.setIsAutoTurning(true);
        s_Swerve.setTurnControllerGoal(s_Swerve.getHeadingToSpeaker());
    }

    @Override
    public void execute() {
        LimelightHelpers.LimelightResults limelightResults = LimelightHelpers.getLatestResults("");
        double latency = limelightResults.targetingResults.latency_capture;
        double newHeading;
        if(!hasSeenTarget && s_VisionSubsystem.hasTarget()) {
            newHeading = s_Swerve.getHeadingByTimestamp(Timer.getFPGATimestamp() - latency/1000) - limelightResults.targetingResults.targets_Fiducials[0].tx;
        } else {
            newHeading = s_Swerve.getHeadingToSpeaker();
        }
        s_Swerve.setAutoTurnHeading(newHeading);
        SmartDashboard.putBoolean("Is Facing Speaker", isFacingSpeaker());
    }

    public boolean isFacingSpeaker() {
        if(s_Swerve.isFacingTurnTarget()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setIsAutoTurning(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFacingSpeaker();
    }
}
