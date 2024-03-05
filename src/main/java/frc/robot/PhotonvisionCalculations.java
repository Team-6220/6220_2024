// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class PhotonvisionCalculations {
    public static AprilTagFieldLayout aprilFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static PhotonCamera[] cameras = {new PhotonCamera("Arducam_OV2311_USB_Camera_0"), new PhotonCamera("Arducam_OV2311_USB_Camera_1")};
    // public PhotonPipelineResult cameraZeroResults, cameraOneResults;
    public static Transform3d camToCenterRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));//Cam mounted facing forward, half a meter forward of center, half a meter up from center. //TODO: need change
    public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[0], camToCenterRobot);
    public ShuffleboardTab visionTab = Shuffleboard.getTab("Photonvision");
    
    public static void updatePoseEstimationCamera1(SwerveDrivePoseEstimator poseEstimator, Swerve s_Swerve)
    {
        Pose2d botPose2d = updatePhotonRobotPoseCamZero(cameras[0].getLatestResult(), cameras[1].getLatestResult(), poseEstimator, s_Swerve.visionMeasurementStdDevConstant.get());
        cameras[0].setPipelineIndex(0);
        cameras[1].setPipelineIndex(0);
        if(botPose2d == null)
        {
            return;
        }

        double camZeroLatency = cameras[0].getLatestResult().getLatencyMillis(), camOneLatency = cameras[1].getLatestResult().getLatencyMillis();
        double averageCamLatency = (camZeroLatency + camOneLatency)/2;
        
        Translation2d botTranslation2d = botPose2d.getTranslation();


        poseEstimator.addVisionMeasurement(new Pose2d(botTranslation2d, s_Swerve.getHeading()), Timer.getFPGATimestamp() - (averageCamLatency/1000));

        Shuffleboard.getTab("Photonvision").add("camera zero latency in ms", camZeroLatency);
        Shuffleboard.getTab("Photonvision").add("camera one latency in ms", camOneLatency);
        Shuffleboard.getTab("Photonvision").add("average camera latency in ms", averageCamLatency);
    }

    public static Pose2d updatePhotonRobotPoseCamZero(PhotonPipelineResult cameraZeroResults, PhotonPipelineResult cameraOneResults, SwerveDrivePoseEstimator poseEstimator, double constant)
    {
        
        int count = 0;
        double totalX = 0;
        double totalY = 0;
        double totalAngle = 0;
        double totalDistance = 0;

        
           
            if(cameraZeroResults.hasTargets())
            {
                 List<PhotonTrackedTarget> targetsCamZero = cameraZeroResults.getTargets();
                 Shuffleboard.getTab("Photonvision").add("camera zero targets", targetsCamZero.size());
                for(PhotonTrackedTarget target : targetsCamZero)
                {
                    Transform3d aprilTagPoseEstimate = target.getAlternateCameraToTarget();

                    totalX += aprilTagPoseEstimate.getX();
                    totalY += aprilTagPoseEstimate.getY();
                    totalAngle += aprilTagPoseEstimate.getRotation().getAngle();
                    totalDistance += aprilTagPoseEstimate.getTranslation().getDistance(new Translation3d(0,0,0));
                    count ++;
                }
            }
            if(cameraOneResults.hasTargets())
            {
                List<PhotonTrackedTarget> targetsCamOne = cameraOneResults.getTargets();
                Shuffleboard.getTab("Photonvision").add("camera one targets", targetsCamOne.size());
                for(PhotonTrackedTarget target : targetsCamOne)
                {
                    Transform3d aprilTagPoseEstimate = target.getAlternateCameraToTarget();

                    totalX += aprilTagPoseEstimate.getX();
                    totalY += aprilTagPoseEstimate.getY();
                    totalAngle += aprilTagPoseEstimate.getRotation().getAngle();
                    totalDistance += aprilTagPoseEstimate.getTranslation().getDistance(new Translation3d(0,0,0));
                    count ++;
                }
            }

            if(count > 0)
            {
                double distance = totalDistance/count;
                Shuffleboard.getTab("Photonvision").add("Distance", distance);
                if(distance > 4)
                {
                    return null;
                }
                else
                {
                    double visionStdDev = constant * (1+ (distance*distance / 30));
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev, visionStdDev, Double.MAX_VALUE));
                }

                Pose2d newPose = new Pose2d(new Translation2d((totalX/count) + VisionConstants.CENTER_OF_FIELD.getX(), (totalY/count) + VisionConstants.CENTER_OF_FIELD.getY()), poseEstimator.getEstimatedPosition().getRotation());
                return newPose;
            }

            
        return null;
    }

    /*
     * public static void updatePoseEstimation(SwerveDrivePoseEstimator poseEstimator, Swerve s_Swerve) {
        LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT3_NAME_STRING, 1);
        LimelightResults limelightResults = LimelightHelpers.getLatestResults(VisionConstants.LIMELIGHT3_NAME_STRING);
        
        if(limelightResults.targetingResults.targets_Fiducials == null) {
            //System.out.println("No Fiducial Targets");
            return;
        }

        
        Pose2d botPose2d = updateVisionRobotPose2d(limelightResults, poseEstimator, s_Swerve.visionMeasurementStdDevConstant.get());
        if(botPose2d == null) {
            return;
        }

        double latency = limelightResults.targetingResults.latency_capture + limelightResults.targetingResults.latency_jsonParse;
        
        Translation2d botTranslation2d = botPose2d.getTranslation();


        poseEstimator.addVisionMeasurement(new Pose2d(botTranslation2d, s_Swerve.getHeading()), Timer.getFPGATimestamp() - (latency/1000.0));
          
     */
}
