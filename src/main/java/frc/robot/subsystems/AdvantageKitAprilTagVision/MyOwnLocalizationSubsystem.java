// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AdvantageKitAprilTagVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AdvantageKitSwerve.Drive;

public class MyOwnLocalizationSubsystem {
  /** Creates a new MyOwnLocalizationSubsystem. */
  public static AprilTagVisionIO[] visionIOs ;//LEFT IS ID 0, RIGHT IS ID 1
  // = {
  //   new AprilTagVisionIOArduCamPhoton("Right_Ardu_Cam", VisionConstants.camerasToCenter[0]),
  //   new AprilTagVisionIOArduCamPhoton("Left_Ardu_Cam", VisionConstants.camerasToCenter[1])
  // };


  public static AprilTagVisionIO.AprilTagVisionIOInputs[] visionInputs;
  public static Pose3d[] estimatedRobotPoses;
  public MyOwnLocalizationSubsystem(AprilTagVisionIO[] visionIOs) {
    MyOwnLocalizationSubsystem.visionIOs = visionIOs;

    estimatedRobotPoses = new Pose3d[visionIOs.length];
    visionInputs = new AprilTagVisionIO.AprilTagVisionIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new AprilTagVisionIO.AprilTagVisionIOInputs();
    }
  }

  public static void updateCamerasPoseEstimation(Drive drive, SwerveDrivePoseEstimator poseEstimator, double camTrustValue)
  {
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
      xSum /= numOfCams;
      ySum /= numOfCams;

      poseEstimator
  }
}
