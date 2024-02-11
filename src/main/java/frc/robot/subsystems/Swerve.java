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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
// import com.pathplanner.lib.*;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private boolean isAutoTurning;
    private ProfiledPIDController turnPidController;

    public ShuffleboardTab fieldPoseTab = Shuffleboard.getTab("Field Pose 2d tab (map)");

    public Field2d field2d = new Field2d();

    private double lastTurnUpdate;
    private double autoTurnHeading;

    private final TunableNumber turnKP = new TunableNumber("turn kP", Constants.SwerveConstants.turnKP);
    private final TunableNumber turnKI = new TunableNumber("turn kI", Constants.SwerveConstants.turnKI);
    private final TunableNumber turnKD = new TunableNumber("turn Kd", Constants.SwerveConstants.turnKD);
    private final TunableNumber turnMaxVel = new TunableNumber("turn MaxVel", Constants.SwerveConstants.turnMaxVel);
    private final TunableNumber turnMaxAccel = new TunableNumber("turn Accel", Constants.SwerveConstants.turnMaxAccel);

    private SwerveModulePosition[] positions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    private final SwerveDriveOdometry odometer;

    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200);


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod3.constants),
            new SwerveModule(1, SwerveConstants.Mod2.constants),
            new SwerveModule(2, SwerveConstants.Mod1.constants),
            new SwerveModule(3, SwerveConstants.Mod0.constants)
        };

        odometer = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(0), positions);

            AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(SwerveConstants.translation_kP, SwerveConstants.translation_kI, SwerveConstants.translation_kD), // Translation PID constants
                    new PIDConstants(SwerveConstants.rotation_kP, SwerveConstants.rotation_kI, SwerveConstants.rotation_kD), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
            );

            // Set up custom logging to add the current path to a field 2d widget
            PathPlannerLogging.setLogActivePathCallback((poses) -> field2d.getObject("path").setPoses(poses));
            Shuffleboard.getTab("Field Pose 2d tab (map)").add("Field 2d", field2d);
            // SmartDashboard.putData("Field", field2d);

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
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }
    /**
     * Resets the odometer value
     */
    public void resetOdometry(Pose2d pose2d){
        System.out.println("Reset Odometry: " + pose2d.getX() + ", " + pose2d.getY());
        this.positions[0] = new SwerveModulePosition();
        this.positions[1] = new SwerveModulePosition();
        this.positions[2] = new SwerveModulePosition();
        this.positions[3] = new SwerveModulePosition();
        odometer.resetPosition(getGyroYaw(), positions, pose2d);
  }

/**
 * Get's the chassis speed of the robot in ROBOT RELATIVE SPEED
 */

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds chassisSpeeds = SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        return chassisSpeeds;
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
        return odometer.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometer.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public double getHeadingToSpeaker(){
        Pose2d currPose = odometer.getPoseMeters();
        double angle = Math.toDegrees(Math.atan2(currPose.getX(), currPose.getY() - 5.5));
        return 180 - angle;
    }

    public double getHeadingDegrees()
    {
        return getPose().getRotation().getDegrees();
    }

    public void setHeading(Rotation2d heading){
        odometer.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        odometer.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    public void setTurnControllerGoal(double goal) {
        turnPidController.setGoal(goal);
    }

    public double getTurnPidSpeed() {

        turnPidController.setGoal(autoTurnHeading);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            resetTurnController();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();

    
        double speed = turnPidController.calculate(getHeading().getDegrees());

        //SmartDashboard.putNumber("raw speed", speed);

        if(speed > SwerveConstants.maxAngularVelocity) {
            speed = SwerveConstants.maxAngularVelocity;
        } if (speed < -SwerveConstants.maxAngularVelocity) {
            speed = -SwerveConstants.maxAngularVelocity;
        }
        return speed;
    }

    @Override
    public void periodic(){
        odometer.update(getGyroYaw(), getModulePositions());
        field2d.setRobotPose(getPose());
        SmartDashboard.putString("getpose", getPose().toString());
        SmartDashboard.putString("getRobotPoseField 2d", field2d.getRobotPose().toString());
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