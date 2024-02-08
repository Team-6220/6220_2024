package frc.robot.subsystems;

import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private boolean isAutoTurning;
    private ProfiledPIDController turnPidController;

    private final TunableNumber turnKP = new TunableNumber("turn kP", Constants.SwerveConstants.turnKP);
    private final TunableNumber turnKI = new TunableNumber("turn kI", Constants.SwerveConstants.turnKI);
    private final TunableNumber turnKD = new TunableNumber("turn Kd", Constants.SwerveConstants.turnKD);
    private final TunableNumber turnMaxVel = new TunableNumber("turn MaxVel", Constants.SwerveConstants.turnMaxVel);
    private final TunableNumber turnMaxAccel = new TunableNumber("turn Accel", Constants.SwerveConstants.turnMaxAccel);

    private final TunableNumber startingX = new TunableNumber("Swerve Starting Position X (Meters)", 0);
    private final TunableNumber startingY = new TunableNumber("Swerve Starting Position Y (Meters)", 0);
    private final TunableNumber startingHeading = new TunableNumber("Starting Position Heading (Degrees): ", 0);

    private double lastTurnUpdate;
    private double autoTurnHeading;

    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod3.constants),
            new SwerveModule(1, SwerveConstants.Mod2.constants),
            new SwerveModule(2, SwerveConstants.Mod1.constants),
            new SwerveModule(3, SwerveConstants.Mod0.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(
            SwerveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(
                startingX.get(),
                startingY.get(),
                Rotation2d.fromDegrees(startingHeading.get())
            )
        );

        turnPidController = new ProfiledPIDController(turnKP.get(), turnKI.get(), turnKD.get(), new TrapezoidProfile.Constraints(turnMaxVel.get(), turnMaxAccel.get()));
        turnPidController.setIZone(Constants.SwerveConstants.turnIZone);
        turnPidController.setTolerance(Constants.SwerveConstants.turnTolerance);
        turnPidController.enableContinuousInput(0, 360);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            SmartDashboard.putString("Mod " + mod.moduleNumber +" Swerve Module State", swerveModuleStates[mod.moduleNumber].toString());
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(){
        swerveOdometry = new SwerveDriveOdometry(
            SwerveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(
                startingX.get(),
                startingY.get(),
                Rotation2d.fromDegrees(startingHeading.get())
            )
        );
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public double getHeadingToSpeaker(){
        Pose2d currPose = swerveOdometry.getPoseMeters();
        //make the speaker the origin, positive x direction is forward to driver, positive y is to the right of the driver for blue and left of the driver for red
        double angle = Math.toDegrees(Math.atan2(currPose.getX(), currPose.getY()));
        return 180 - angle;
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public boolean isAutoTurning() {
        return isAutoTurning;
    }

    public boolean isFacingTurnTarget() {
        return turnPidController.atGoal();
    }

    public void setIsAutoTurning(boolean state) {
        isAutoTurning = state;
    }

    public void setAutoTurnHeading(double heading) {
        autoTurnHeading = heading;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void resetTurnController() {
        turnPidController.reset(getHeading().getDegrees());
        System.out.println("ResetTurnController");
    }

    public double getTurnPidSpeed() {

        turnPidController.setGoal(autoTurnHeading);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            resetTurnController();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();

    
        double speed = turnPidController.calculate(getHeading().getDegrees());

        SmartDashboard.putNumber(" raw speed", speed);

        if(speed > SwerveConstants.maxAngularVelocity) {
            speed = SwerveConstants.maxAngularVelocity;
        } if (speed < -SwerveConstants.maxAngularVelocity) {
            speed = -SwerveConstants.maxAngularVelocity;
        }
        return speed;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "setAngle", mod.getDesiredState());
        }

        SmartDashboard.putNumber("Real Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Auto Turn Heading", autoTurnHeading);
        SmartDashboard.putNumber("Turn Controller Setpoint", turnPidController.getSetpoint().position);
        if(turnKP.hasChanged()
        || turnKD.hasChanged()
        || turnKI.hasChanged()) {
            turnPidController.setPID(turnKP.get(), turnKI.get(), turnKD.get());
            turnPidController.reset(getHeading().getDegrees());
        }
        if(turnMaxAccel.hasChanged() || turnMaxVel.hasChanged()) {
            turnPidController.setConstraints(new TrapezoidProfile.Constraints(turnMaxVel.get(), turnMaxAccel.get()));
            turnPidController.reset(getHeading().getDegrees());
        }
        
    }
}