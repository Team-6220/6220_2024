// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult currResult;
  private boolean hasTargets;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private double pitch, yaw, skew, area;

  private static VisionSubsystem INSTANCE = null;

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
    this.camera = new PhotonCamera("photonvision");
    currResult = camera.getLatestResult();
    hasTargets = currResult.hasTargets();
  }

  public static VisionSubsystem getInstance(){
    if(INSTANCE == null)
      INSTANCE = new VisionSubsystem();
    return INSTANCE;
  }

  private void updateValues(){
    currResult = camera.getLatestResult();
    hasTargets = currResult.hasTargets();
    targets = currResult.getTargets();
    bestTarget = currResult.getBestTarget();
    yaw = bestTarget.getYaw();
    pitch = bestTarget.getPitch();
    skew = bestTarget.getSkew();
    area = bestTarget.getArea();
  }

  @Override
  public void periodic() {
    updateValues();
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("yaw", yaw);
  }
}
