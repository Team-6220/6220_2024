package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ShooterConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.blinkin;

public class SpeakerCommand extends Command{
    private final Swerve swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final XboxController driver;
    private double[] velocities  = {0, 0};
    private double armAngle = ArmConstants.restingSetpoint;
    private boolean isAuto, hasFired;
    private blinkin s_Blinkin;
    private double shotClock;
    private ShooterConfiguration currentShooterConfiguration;

    public SpeakerCommand(Swerve swerve, XboxController driver){
        isAuto = false;
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.s_Blinkin = blinkin.getInstance();
        this.driver = driver;
        addRequirements(this.swerve, arm, intake, shooter, s_Blinkin);
    }

    public SpeakerCommand(Swerve swerve){
        isAuto = true;
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.s_Blinkin = blinkin.getInstance();
        this.driver = null;
        intake.setHasNote();
        addRequirements(this.swerve, arm, intake, shooter, s_Blinkin);
    }

    @Override
    public void initialize() {
        hasFired = false;
        shotClock = 0;
        
    } 

    @Override
    public void execute(){

        
        try {
            currentShooterConfiguration = ShooterConfiguration.getShooterConfiguration(swerve.getPose());
        } catch(Exception e) {
            System.out.println(e);
            end(true);
            
        }
        
        if(currentShooterConfiguration != null) {
            System.out.println("Is not null!");
            double[] driverInputs;
            if(!isAuto)
            {
                driverInputs = OIConstants.getDriverInputs(driver);
            }
            else
            {
                driverInputs = new double[] {0,0,0};
            }
            double teamOffset = Constants.isRed ? 0 + currentShooterConfiguration.getHeadingOffset() : 180 + currentShooterConfiguration.getHeadingOffset();
            swerve.setAutoTurnHeading(swerve.getHeadingToSpeaker() +teamOffset);
            double rotationVal = swerve.getTurnPidSpeed();
            swerve.drive(
                    new Translation2d(driverInputs[0], driverInputs[1]), 
                    rotationVal, 
                    true, 
                    true
            );

            //Pose2d currPose = swerve.getPose();
            //Pose2d speakerPose = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;

            
            //double distanceToSpeaker = Math.hypot(currPose.getX()-speakerPose.getX(), currPose.getY()-speakerPose.getY());

            //velocities = ShooterConstants.getVelocitiesFromDistance(distanceToSpeaker);
            shooter.spinToVelocity(currentShooterConfiguration.getVelocities());

            //armAngle = ArmConstants.getArmAngleFromDistance(distanceToSpeaker);
            arm.driveToGoal(currentShooterConfiguration.getArmAngle());
            
            if((shooter.isAtSetpoint() && swerve.isFacingTurnTarget() && arm.isAtGoal()) || hasFired){
                intake.feedShooter();
                hasFired = true;
                shotClock++;
                s_Blinkin.solid_green();
            } else if(!swerve.isFacingTurnTarget()) {
                s_Blinkin.solid_red();
            } else if(!arm.isAtGoal()) {
                s_Blinkin.solid_purple();
            } else if(!shooter.isAtSetpoint()) {
                s_Blinkin.solid_blue();
            }
            
        }
        
    }

    @Override
    public boolean isFinished() {
        if(shotClock > ShooterConstants.fireTime*50) {
            return true;
        }
    // if(!intake.noteInIntake()) {
    //     end(true);
    // }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
        shooter.stop();
    }

}
