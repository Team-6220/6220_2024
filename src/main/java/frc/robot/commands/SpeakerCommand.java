package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class SpeakerCommand extends Command{
    private final Swerve swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final XboxController driver;
    private final BooleanSupplier manualOverride;
    private double[] velocities  = {0, 0};
    private double armAngle = ArmConstants.restingSetpoint;
    public SpeakerCommand(Swerve swerve, XboxController driver, BooleanSupplier override){
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.driver = driver;
        this.manualOverride = override;
        addRequirements(this.swerve, arm, intake, shooter);
    }

    @Override
    public void execute(){
        double[] driverInputs = OIConstants.getDriverInputs(driver);
        swerve.setAutoTurnHeading(swerve.getHeadingToSpeaker());
        double rotationVal = swerve.getTurnPidSpeed();
        if(manualOverride.getAsBoolean()){
            rotationVal =  driverInputs[2];
        }
        swerve.drive(
                new Translation2d(driverInputs[0], driverInputs[1]), 
                rotationVal, 
                true, 
                true
        );

        Pose2d currPose = swerve.getPose();
        Pose2d speakerPose = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;

        double distanceToSpeaker = Math.hypot(currPose.getX()-speakerPose.getX(), currPose.getY()-speakerPose.getY());

        velocities = ShooterConstants.getVelocitiesFromDistance(distanceToSpeaker);
        shooter.spinToVelocity(velocities);

        armAngle = ArmConstants.getArmAngleFromDistance(distanceToSpeaker);
        arm.driveToGoal(armAngle);
        if(shooter.isAtSetpoint() && (swerve.isFacingTurnTarget() || manualOverride.getAsBoolean()) && arm.isAtGoal()){
            intake.feedShooter();
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
        shooter.stop();
    }

}
