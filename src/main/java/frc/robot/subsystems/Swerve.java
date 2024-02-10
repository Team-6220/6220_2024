package frc.robot.subsystems;

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
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

// import com.pathplanner.lib.*;

import frc.lib.util.TunableNumber;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

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

        

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        odometer = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(0), positions);

            AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
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
        this.positions[0] = new SwerveModulePosition();
        this.positions[1] = new SwerveModulePosition();
        this.positions[2] = new SwerveModulePosition();
        this.positions[3] = new SwerveModulePosition();
        odometer.resetPosition(getGyroYaw(), positions, pose2d);
  }

/**
 * Get's the chassis speed of the robot in ROBOT RELATIVE SPEED
 */

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
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
        Pose2d afterModify = swerveOdometry.getPoseMeters();
        
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public double getHeadingDegrees()
    {
        return getPose().getRotation().getDegrees();
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

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putString("getpose", getPose().toString());
        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "setAngle", mod.getDesiredState());
        }
    }
}