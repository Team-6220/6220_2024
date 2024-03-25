package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ShooterConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
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
    private boolean isAuto, hasFired, armReady, shooterReady, aimingReady, noteReady;
    private blinkin s_Blinkin;
    private double startShotTimeStamp, fireShotTimeStamp, armTime, shooterTime, aimingTime, noteTime;
    private static int shotCount = 0;
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
        startShotTimeStamp = Timer.getFPGATimestamp();
        resetStatusBooleans();
    } 

    private void resetStatusBooleans() {
        armReady = false;
        shooterReady = false;
        aimingReady = false;
        noteReady = false;
        hasFired = false;
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
            
            if(conditionsMet() || hasFired){
                intake.feedShooter();
                s_Blinkin.solid_green();
            } else if(!swerve.isFacingTurnTarget()) {
                s_Blinkin.solid_red();
            } else if(!arm.isAtGoal()) {
                s_Blinkin.solid_purple();
            } else if(!shooter.isAtSetpoint()) {
                s_Blinkin.solid_blue();
            }
            
        } else {
            end(false);
        }
        
    }
    private boolean conditionsMet() {
        if(shooter.isAtSetpoint() && !shooterReady) {
            shooterReady = true;
            shooterTime = Timer.getFPGATimestamp();
        }
        if(arm.isAtGoal() && !armReady) {
            armReady = true;
            armTime = Timer.getFPGATimestamp();
        }
        if(intake.noteReady() && !noteReady) {
            noteReady = true;
            noteTime = Timer.getFPGATimestamp();
        }
        if(swerve.isFacingTurnTarget() && !aimingReady) {
            aimingReady = true;
            aimingTime = Timer.getFPGATimestamp();
        }
        if((shooterReady && armReady && noteReady && aimingReady) || hasFired) {
            if(!hasFired) {
                hasFired = true;
                fireShotTimeStamp = Timer.getFPGATimestamp();
            } 
            return true;
        }
        return false;
    }
    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        if(hasFired && currentTime-fireShotTimeStamp > ShooterConstants.fireTime) {
            String shotLog = 
                "Shot #" + shotCount + 
                "\nTime To Shot: " + (startShotTimeStamp - startShotTimeStamp) + 
                "\nArm Time: " + (armTime  - startShotTimeStamp) + 
                "\nShooter Time: " + (shooterTime  - startShotTimeStamp) + 
                "\nAiming Time: " + (aimingTime - startShotTimeStamp) +
                "\nIntakeSet Time: " + (noteTime - startShotTimeStamp);
            System.out.println(shotLog);
            return true;
        }
        else if(isAuto && AutoConstants.currentCenterNotePos > AutoConstants.howManyNotesAreWeAttempting)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
        shooter.stop();
    }

}
