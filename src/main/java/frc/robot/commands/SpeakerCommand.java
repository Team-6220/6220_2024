package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double startShotTimeStamp, fireShotTimeStamp, armTime, shooterTime, aimingTime, noteTime;
    private static int shotCount = 0;
    private ShooterConfiguration currentShooterConfiguration;



    public SpeakerCommand(Swerve swerve, XboxController driver){
        isAuto = false;
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.driver = driver;
        addRequirements(this.swerve, arm, intake, shooter);
    }

    public SpeakerCommand(Swerve swerve){
        isAuto = true;
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.driver = null;
        intake.setHasNote();
        addRequirements(this.swerve, arm, intake, shooter);
    }

    @Override
    public void initialize() {
        startShotTimeStamp = Timer.getFPGATimestamp();
        hasFired = false;
        resetStatusBooleans();
    } 

    private void resetStatusBooleans() {
        armReady = false;
        shooterReady = false;
        aimingReady = false;
        noteReady = false;
        hasFired = false;
        armTime = 0;
        aimingTime = 0;
        shooterTime = 0;
        noteTime = 0;
    
    }
    @Override
    public void execute(){
        
        // System.out.println(hasFired);
        
        try {
            currentShooterConfiguration = ShooterConfiguration.getShooterConfiguration(swerve.getPose());
        } catch(Exception e) {
            System.out.println(e);
            end(true);
            
        }
        
        if(currentShooterConfiguration != null) {
            double teamOffset = Constants.isRed ? 0 + currentShooterConfiguration.getHeadingOffset() : 180 + currentShooterConfiguration.getHeadingOffset();

            double[] driverInputs;
            if(!isAuto)
            {
                driverInputs = OIConstants.getDriverInputs(driver);
            }
            else
            {
                driverInputs = new double[] {0,0,0};
            }
            swerve.setAutoTurnHeading((swerve.getHeadingToSpeaker() +teamOffset));
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
            arm.driveToGoal(currentShooterConfiguration.getArmAngle() + ArmConstants.armDegreesOffset);
            
            if(swerve.isFacingTurnTarget()&& arm.isAtGoal()&& shooter.isAtSetpoint() || hasFired){
                intake.feedShooter();
                if(!hasFired) {
                    hasFired = true;
                    fireShotTimeStamp = Timer.getFPGATimestamp();
                }
            } 
            SmartDashboard.putBoolean("Facing Turn Heading", swerve.isFacingTurnTarget());
            SmartDashboard.putNumber("Turn Target", (swerve.getHeadingToSpeaker() +teamOffset));
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
        // if(intake.noteReady() && !noteReady) {
        //     noteReady = true;
        //     noteTime = Timer.getFPGATimestamp();
        // }
        noteReady = true;
        if(swerve.isFacingTurnTarget() && !aimingReady) {
            aimingReady = true;
            aimingTime = Timer.getFPGATimestamp();
        }
        if((shooterReady && armReady && noteReady && aimingReady) || hasFired) {
            if(!hasFired) {
                hasFired = true;
                shotCount++;
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
            hasFired = false;
            resetStatusBooleans();
            return true;
        }
        // else if(isAuto && AutoConstants.currentCenterNotePos > AutoConstants.howManyNotesAreWeAttempting)
        // {
            // return true;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        hasFired = false;
        arm.stop();
        intake.stop();
        shooter.stop();
        // String shotLog = 
        //         "Shot #" + shotCount + 
        //         "\nTime To Shot: " + (startShotTimeStamp - startShotTimeStamp) + 
        //         "\nArm Time: " + (armTime  - startShotTimeStamp) + 
        //         "\nShooter Time: " + (shooterTime  - startShotTimeStamp) + 
        //         "\nAiming Time: " + (aimingTime - startShotTimeStamp) +
        //         "\nIntakeSet Time: " + (noteTime - startShotTimeStamp);
        //     System.out.println(shotLog);
        resetStatusBooleans();
    }

}
