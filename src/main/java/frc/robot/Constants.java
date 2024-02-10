// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static boolean TUNING_MODE = true;

    public static Optional<DriverStation.Alliance> ALLIANCE_COLOR = DriverStation.getAlliance();

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;

        public static double modifyMoveAxis(double value) {
            // Deadband
            if(Math.abs(value) < kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .1;
            double a = .5;
            double x = value;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            } else{
                return -b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            }
            //value = Math.copySign(value * value, value);
        
            //return value;
          }
        public static double modifyRotAxis(double value) {
            // Deadband
            if(Math.abs(value) < kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .05;
            double a = .2;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            } else{
                return -b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            }
          }
    }

    public static final class ArmConstants{
        //FIXME: set ids
        public static final int armMotorAID = 0;
        public static final int armMotorBID = 0;

        //FIXME: set inverted
        public static final boolean motorAInverted = false;
        public static final boolean motorBInverted = false;

        //FIXME: set pid values
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;

        //FIXME: create lookup table
        public static final double [][] armLookupTable = {
            {/* distance to target, arm angle */}
        };

        //FIXME: set setpoints
        public static final double intakeSetpoint = 0;
        public static final double restingSetpoint = 0;
    }

    public static final class IntakeConstants{
        //FIXME: set id
        public static final int intakeMotorID = 0;

        //FIXME: set break beam port
        public static final int breakBeamPort = 0;

        //FIXME: set intake speed
        public static final double intakeSpeed = 0;
    }

    public static final class ShooterConstants{
        //FIXME: set motor IDs
        public static final int shooterMotorAID = 0;
        public static final int shooterMotorBID = 0;

        //FIXME: set break beam port
        public static final int breakBeamPort = 0;

        //FIXME: set shooter velocity pid
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;

        //FIXME: create lookup table
        public static final double [][] shooterLookupTable = {
            {/* distance to target, shooter speed */}
        };

    }

    public static final class PhotonVisionConstants{
        // Constants such as camera and target height stored. Change per robot and goal!
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24); //TODO: DELTE AFTER DONE -- Change this accoridngly
        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5); //TODO: DELTE AFTER DONE -- Change this accoridngly
        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); //TODO: DELTE AFTER DONE -- Change this accoridngly

        // How far from the target we want to be
        
        // PID constants should be tuned per robot
        // public static final double LINEAR_P = 0.1;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_I = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_D = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_IZONE = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_MAXIMUMACCEL = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_MAXVEL = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double LINEAR_Tolerance = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly

        // public static final double ANGULAR_P = 0.1;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_I = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_D = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_IZONE = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_MAXIMUMACCEL = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_MAXVEL = 0.0; //TODO: DELTE AFTER DONE -- Change this accoridngly
        // public static final double ANGULAR_Tolerance = 0.0;//TODO: DELTE AFTER DONE -- Change this accoridngly

    }

    public static final class VisionConstants{
        //FIXME: set limelight values
        public static final double limelightHeightInches = 0;
        public static final double limelightAngleDegrees = 0;

        public static HashMap<Integer, Double> tagHeights = new HashMap<Integer, Double>();
        
        public static void setTagHeights(){
            tagHeights.put(1, 48.125);
            tagHeights.put(2, 48.125);
            tagHeights.put(9, 48.125);
            tagHeights.put(10, 48.125);
            tagHeights.put(3, 53.875);
            tagHeights.put(4, 53.875);
            tagHeights.put(7, 53.875);
            tagHeights.put(8, 53.875);
            tagHeights.put(5, 53.125);
            tagHeights.put(6, 53.125);
            tagHeights.put(11, 47.5);
            tagHeights.put(12, 47.5);
            tagHeights.put(13, 47.5);
            tagHeights.put(14, 47.5);
            tagHeights.put(15, 47.5);
            tagHeights.put(16, 47.5);
        }

        public static final double speakerTagID = ALLIANCE_COLOR.isPresent()
                                            ?
                                                ALLIANCE_COLOR.get() == DriverStation.Alliance.Red
                                                ?
                                                    4d
                                                :
                                                    7d
                                            :
                                                -1d;
                                                     
    }

    public static final class SwerveConstants {
        public static final int pigeonID = 1;

        public static final double translation_kP = 2.518;
        public static final double translation_kI = 0.6;
        public static final double translation_kD = 0.0;
        public static final double rotation_kP = 1.35;
        public static final double rotation_kI = 1.25;
        public static final double rotation_kD = 0.0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.driveMotorInvert == InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.5;
        public static final double angleKI = 0;
        public static final double angleKD = 0.15;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;


        //Turning Pid Constants
        public static final double turnKP = 6;
        public static final double turnKD = 0;
        public static final double turnKI = 1;
        public static final double turnMaxVel = 400;
        public static final double turnMaxAccel = 800;
        public static final double turnTolerance = 2;
        public static final double turnIZone = 1;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 6380.0 / 60.0 * wheelCircumference * driveGearRatio;
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        // Back Right Module 0
        public static final class Mod0 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(103.71);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Left Module 1
        public static final class Mod1 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-57);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        //Front Right - Module 2
        public static final class Mod2 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(97.12);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //Front left Module 3
        public static final class Mod3 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-41.74);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //FIXME: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ClimberConstants{
        //FIXME: set motor IDs
        public static final int climberDriverLeftID = 0;
        public static final int climberDriverRightID = 0;

        //FIXME: set setpoints
        public static final double topSetpoint = 0;
        public static final double bottomSetpoint = 0;
        public static final double climbedSetpoint = 0;

        //FIXME: set PID constants
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
