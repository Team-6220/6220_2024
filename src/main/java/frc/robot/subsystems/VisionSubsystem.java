// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem INSTANCE = null;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tx = table.getEntry("tx"); //horizontal offset
  NetworkTableEntry ty = table.getEntry("ty"); //vertical offset
  NetworkTableEntry ta = table.getEntry("ta"); //Target area (0% of image to 100% of image)
  NetworkTableEntry tl = table.getEntry("tl"); //The pipeline's latency contribution (ms). Add to "cl" to get total latency.
  NetworkTableEntry tv = table.getEntry("tv"); //check whetehr the limelight have any valid target
  
  // private final PhotonCamera camera;
  // private PhotonPipelineResult currResult;
  // private boolean hasTargets;
  // private List<PhotonTrackedTarget> targets;
  // private PhotonTrackedTarget bestTarget;
  // private double pitch, yaw, skew, area;

  
  double xOffset = tx.getDouble(0.0);
  double yOffset = ty.getDouble(0.0);
  double validTarget = tv.getDouble(0.0);
  
  // ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
  // Check https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api for details
  
  private double distanceFromLimelightToGoalInches;
  
  double limelightMountAngleDegrees = 0;// TODO:GET THIS INTO CONSTANT + CHANGE ACCORDINGLY
  double limelightLensHeightInches = 20;// TODO:GET THIS INTO CONSTANT + CHANGE ACCORDINGLY 
  
  // Check https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
  private double targetHeight = 0; //TODO: CHANGE ACCORDINGLY -- The height of the target, property of the field? (h2 in the equation in the link)
  private double heightOfCamAboveFloor = 0; //TODO: CHANGE ACCORDINGLY -- The height of your camera above the floor (h1 in the equation in the link)
  
  
  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
    // this.camera = new PhotonCamera("photonvision");
    //   currResult = camera.getLatestResult();
    //   hasTargets = currResult.hasTargets();
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");

  //  tab.add("tx", tx);
  //  tab.add("ty", ty);
  //  tab.add("ta- area", ta);
  //  tab.add("tl latency", tl);
  //  tab.add("tv- boolean target", tv);

  }

  // @Override
  // public void periodic() {
  //   ty = table.getEntry("ty").getDouble(0.0);
  //     double verticalOffset = ty; //add anything if needed
  //     double angle = (verticalOffset+limelightMountAngleDegrees)*(3.14159/180);
  //     distanceFromLimelightToGoalInches = (targetHeight-heightOfCamAboveFloor)/Math.tan(angle);

  // }

  public double getdistanceFromLimelightToGoalInches()
  {
    yOffset = ty.getDouble(0.0);
      double verticalOffset = yOffset; //add anything if needed
      double angle = (verticalOffset+limelightMountAngleDegrees)*(3.14159/180);
      distanceFromLimelightToGoalInches = (targetHeight-heightOfCamAboveFloor)/Math.tan(angle);
    return distanceFromLimelightToGoalInches;
  }

  public double getSteeringOffset()
  {
    xOffset = tx.getDouble(0.0) + 0; //add any offsets if needed
    return xOffset;
  }

  public boolean hasTarget()
  {
    if(tv.getDouble(0.0) == 1) {
      return true;
    } else {
      return false;
    }
  }


  private void updateValues(){
    // currResult = camera.getLatestResult();
    // hasTargets = currResult.hasTargets();
    // targets = currResult.getTargets();
    // bestTarget = currResult.getBestTarget();
    // yaw = bestTarget.getYaw();
    // pitch = bestTarget.getPitch();
    // skew = bestTarget.getSkew();
    // area = bestTarget.getArea();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("testtx ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
      xOffset = tx.getDouble(0.0);
      validTarget = tv.getDouble(0.0);

       SmartDashboard.putNumber("xOffset", xOffset);
       SmartDashboard.putNumber("validTarget", validTarget);
      //  tab.add("tx", tx);
      //  tab.add("ty", ty);
      //  tab.add("ta- area", ta);
      //  tab.add("tl latency", tl);
      //  tab.add("tv- boolean target", tv);
        
    updateValues();
    // SmartDashboard.putNumber("pitch", pitch);
    // SmartDashboard.putNumber("yaw", yaw);
  }

  public static synchronized VisionSubsystem getInstance(){
    if(INSTANCE == null)
      INSTANCE = new VisionSubsystem();
    return INSTANCE;
  }
}