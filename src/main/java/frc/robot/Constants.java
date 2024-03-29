// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Optional;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    
    public static boolean TUNING_MODE = true;

    public static Optional<DriverStation.Alliance> ALLIANCE_COLOR = DriverStation.getAlliance();

    public static boolean isRed = ALLIANCE_COLOR.isPresent() && ALLIANCE_COLOR.get().equals(Alliance.Red);

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;

        public static final int translationAxis = XboxController.Axis.kLeftY.value;
        public static final int strafeAxis = XboxController.Axis.kLeftX.value;
        public static final int rotationAxis = XboxController.Axis.kRightX.value;

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

        public static double[] getDriverInputs(XboxController driver) {
            double[] inputs = new double[3];

            inputs[0] = OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis));
            inputs[1] = OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis));
            inputs[2] = OIConstants.modifyRotAxis(-driver.getRawAxis(rotationAxis));

            inputs[0] = MathUtil.applyDeadband(inputs[0], OIConstants.kDeadband);
            inputs[1] = MathUtil.applyDeadband(inputs[1], OIConstants.kDeadband);
            inputs[2] = MathUtil.applyDeadband(inputs[2], OIConstants.kDeadband);

            int invert =  (Constants.isRed) ? -1 : 1; 

            inputs[0] *= invert;
            inputs[1] *= invert;

            inputs[0] *= SwerveConstants.maxSpeed;
            inputs[1] *= SwerveConstants.maxSpeed;
            inputs[2] *= SwerveConstants.maxAngularVelocity;
            return inputs;
        }
    }

    public static final class ArmConstants{

        public static final int armMotorAID = 13;
        public static final int armMotorBID = 14;

        public static final boolean motorAInverted = true;
        public static final boolean motorBInverted = false;

        //FIXME: set pid values
        public static final double kP = .04; //0.009
        public static final double kI = 0.03;//0.0005
        public static final double kD = 0.0005;//0.001
        public static final double armMaxVel = 65;
        public static final double armMaxAccel = 85;


        public static final double minArmShootAngle = 75;
        public static final double maxArmShootAngle = 40;
        //FIXME: create lookup table
        public static final double [][] armLookupTable = {
            {1, 75},
            {3, 65}
        };

        public static double getArmAngleFromDistance(double distance) {

            if(distance < armLookupTable[0][0]) {
                return minArmShootAngle;
            }
            if(distance > armLookupTable[armLookupTable.length-1][0]) {
                return maxArmShootAngle;
            }

            double[] smaller = new double[2];
            double[] larger = new double[2];

            for(int i = 0; i < armLookupTable.length-1; i++) {
                if(distance >= armLookupTable[i][0] && distance <= armLookupTable[i+1][0]) {
                    smaller = armLookupTable[i];
                    larger = armLookupTable[i+1];
                    break;
                }
            }
            //Y = Y1 + (X - X1) * ((Y2 - Y1)/(X2 - X1))
            return smaller[1] + (distance - smaller[0]) * ((larger[1]-smaller[1])/(larger[0]-smaller[0]));
            
            
        }

        public static final double armOffset = -80; // arm up

        //FIXME: set setpoints
        public static final double intakeSetpoint = 83.5;
        public static final double hoverSetpoint = 0; //for like right above intake
        public static final double restingSetpoint = 70;
        public static final double ampSetPoint = -5;
        public static final double ampShooterSpeed = 0.6; // TODO: change this accordingly

        //FIXME: set actual port values and reversed for arm encoder
        public static final int k_ENC_PORT = 0;
    }

    public static final class IntakeConstants{
        //FIXME: set id
        public static final int intakeMotorID = 15;

        //FIXME: set inverted
        public static final boolean intakeMotorInverted = true;

        //FIXME: set break beam port
        public static final int breakBeamPort = 1;

        //FIXME: set intake speed
        public static final double intakeSpeed = .3;
        public static final double ejectSpeedSpeaker = .95;
        public static final double ejectSpeedAmp = .4;

        public static final double armSetPointIntake = 83.5;

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double transitDistance = 2;
    }

    public static final class ShooterConstants{
        //FIXME: set motor IDs
        public static final int shooterMotorAID = 16;
        public static final int shooterMotorBID = 17;

        public static final boolean motorAInverted = false;
        public static final boolean motorBInverted = false;


        public static final double idleOutput = .05;

        public static final double minShooterVelA = 2000;
        public static final double minShooterVelB = 2000;

        public static final double maxShooterVelA = 4500;
        public static final double maxShooterVelB = 4500;
        //FIXME: set break beam port
        // public static final int breakBeamPort = 0;

        //FIXME: set shooter velocity pid
        public static final double kP = 0.0007;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFFkS = 0;
        public static final double kFFkV = 0.00017;
        public static final double kFFkA = 0;

        //FIXME: create lookup table
        public static final double[][] shooterLookupTable = {
            {3000,3000,1},
            {4000,4000,3}
        };
        public static double[] getVelocitiesFromDistance(double distance) {
            double[] velocities = new double[2];

            if(distance < shooterLookupTable[0][2]) {
                velocities[0] = minShooterVelA;
                velocities[1] = minShooterVelB;
                return velocities;
            }
            if(distance > shooterLookupTable[shooterLookupTable.length-1][2]) {
                velocities[0] = maxShooterVelA;
                velocities[1] = maxShooterVelB;
                return velocities;
            }

            double[] smaller = new double[3];
            double[] larger = new double[3];

            for(int i = 0; i < shooterLookupTable.length-1; i++) {
                if(distance >= shooterLookupTable[i][2] && distance <= shooterLookupTable[i+1][2]) {
                    smaller = shooterLookupTable[i];
                    larger = shooterLookupTable[i+1];
                    break;
                }
            }
            //Y = Y1 + (X - X1) * ((Y2 - Y1)/(X2 - X1))
            velocities[0] = smaller[0] + (distance - smaller[2]) * ((larger[0]-smaller[0])/(larger[2]-smaller[2]));
            velocities[1] = smaller[0] + (distance - smaller[2]) * ((larger[1]-smaller[1])/(larger[2]-smaller[2]));
            return velocities;
        }
    }

    public static final class VisionConstants{

        public static final String LIMELIGHT3_NAME_STRING = "limelight";
        public static final String LIMELIGHT2_NAME_STRING = "Limelight_2";


        public static final Pose2d SPEAKER_POSE2D_BLUE = new Pose2d(new Translation2d(-.0381, 5.547868), new Rotation2d(0));
        public static final Pose2d SPEAKER_POSE2D_RED = new Pose2d(new Translation2d(16.5793, 5.547868), new Rotation2d(180));
        public static final Pose2d AMP_POSE2D_RED = new Pose2d(new Translation2d(Units.inchesToMeters(580.77), Units.inchesToMeters(323-23.25)), new Rotation2d(270));
        public static final Pose2d AMP_POSE2D_BLUE = new Pose2d(new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323-23.25)), new Rotation2d(270));

        
        public static final Translation2d CENTER_OF_FIELD = new Translation2d(8.2706,4.105148);
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.98828125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Left Module 1
        public static final class Mod1 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-156.181640625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        //Front Right - Module 2
        public static final class Mod2 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(79.892578125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //Front left Module 3
        public static final class Mod3 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.751953125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //FIXME: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 10;
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
