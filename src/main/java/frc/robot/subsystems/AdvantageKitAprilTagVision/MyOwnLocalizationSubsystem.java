// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitAprilTagVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AdvantageKitSwerve.Drive;

public class MyOwnLocalizationSubsystem {
  /** Creates a new MyOwnLocalizationSubsystem. */
  public static AprilTagVisionIOArduCamPhoton[] visionIOs = {new AprilTagVisionIOArduCamPhoton("Left_Ardu_cam", VisionConstants.camerasToCenter[0]), new AprilTagVisionIOArduCamPhoton("Right_Ardu_Cam", VisionConstants.camerasToCenter[1])};//LEFT IS ID 0, RIGHT IS ID 1
  // = {
  //   new AprilTagVisionIOArduCamPhoton("Right_Ardu_Cam", VisionConstants.camerasToCenter[0]),
  //   new AprilTagVisionIOArduCamPhoton("Left_Ardu_Cam", VisionConstants.camerasToCenter[1])
  // };

  public static AprilTagVisionIO.AprilTagVisionIOInputs[] visionInputs;
  public static Pose3d[] estimatedRobotPoses;
  public static void initilizeSubsystem()
  {
    estimatedRobotPoses = new Pose3d[visionIOs.length];
    visionInputs = new AprilTagVisionIO.AprilTagVisionIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new AprilTagVisionIO.AprilTagVisionIOInputs();
    }
  }
  //Gotta create a Myownlocaliationsubsystem objectin Drive...
  public MyOwnLocalizationSubsystem() {
    // MyOwnLocalizationSubsystem.visionIOs = visionIOs;

   
  }

  /**
   * @camTrustValue the value how much we trust the cameras, you can change it in drive but we come up with 0.1
   */
  public static void updateCamerasPoseEstimation(Drive drive, SwerveDrivePoseEstimator poseEstimator, double camTrustValue)
  {
    double timestamp = 0;
    double visionStdDev = camTrustValue;
    double movementAddition = 10;
    double nonMultiAddition = 20;
    PoseStrategy[] camerasStrategy = new PoseStrategy[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      if (!Logger.hasReplaySource()) {
        visionIOs[i].updateInputs(visionInputs[i]);
      }

      Logger.processInputs(
          "Estimates/AprilTag/" + visionIOs[i].getCameraName(), visionInputs[i]);
    }
      for(int cameras = 0; cameras < visionInputs.length; cameras ++)
      {
        String instanceLogKey =
          "Estimates/AprilTag/" + visionIOs[cameras].getCameraName() + "/";
          estimatedRobotPoses[cameras] = visionInputs[cameras].estimate.estimatedPose;
          camerasStrategy[cameras] = visionInputs[cameras].estimate.strategy;
           //.....some where over here and then ya somehow need to either divide them by the num of cams or you can decide to abandon the non multi tag pnp on coprocessor by 
        // 
        timestamp += visionInputs[cameras].estimate.timestampSeconds;
        if(visionInputs[cameras].estimate.estimatedPose.getX() == -1)
        {
          estimatedRobotPoses[cameras] = null;
          camerasStrategy[cameras] = null;
        }
        if(visionInputs[cameras].estimate.strategy != null && visionInputs[cameras].estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
          // Reject estimates that are off of the field
        if (estimatedRobotPoses[cameras].getX() < -VisionConstants.fieldBorderMargin
        || estimatedRobotPoses[cameras].getX()
            > VisionConstants.fieldSize.getX() + VisionConstants.fieldBorderMargin
        || estimatedRobotPoses[cameras].getY() < -VisionConstants.fieldBorderMargin
        || estimatedRobotPoses[cameras].getY()
            > VisionConstants.fieldSize.getY() + VisionConstants.fieldBorderMargin
        || estimatedRobotPoses[cameras].getZ() < -VisionConstants.zMargin
        || estimatedRobotPoses[cameras].getZ() > VisionConstants.zMargin) {
          estimatedRobotPoses[cameras] = null;
        }
      }
      double xSum = 0;
      double ySum = 0;
      // Pose3d estimatedRobotPose3dAfterFiltering = new Pose3d(0, 0, 0, new Rotation3d());
      int numOfCams = 0;
      // boolean isMultitag = true;
      for(int i = 0; i < visionInputs.length; i ++)
      {
        if(estimatedRobotPoses[i] == null)
        {
          continue;
        }
       
        xSum += estimatedRobotPoses[i].getX();
        ySum += estimatedRobotPoses[i].getY();
        numOfCams ++;
      }
      xSum = numOfCams != 0 ? (xSum / numOfCams): 0;
      ySum /= numOfCams != 0 ? (ySum / numOfCams): 0;
      timestamp /= numOfCams;
      
      
      if(numOfCams > 0)
      {
        if(drive.getCurrentSpeeds().vxMetersPerSecond > 1 || drive.getCurrentSpeeds().vyMetersPerSecond > 1)
        {
          visionStdDev += movementAddition;
        }
        //Ok for detecting non multi tag pnp on coprocessor ya need to do it ... look up for ..... NO, YA NEED STD DEVIATION...
        // so bascially what ya do is stddev * estimatedrobotpose[i].getx base on if it's multi or not then add them together then divide em.
      }
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev, visionStdDev, Double.MAX_VALUE));
      poseEstimator.addVisionMeasurement(new Pose2d(new Translation2d(xSum,ySum), new Rotation2d()), timestamp );//addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3,N1> visionMeasurementStdDevs) : void
  }
}
