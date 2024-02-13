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

import org.opencv.core.Mat.Tuple2;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem INSTANCE = null;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tx,ty,ta, tl, tv, jsonDumpEntry;//check whetehr the limelight have any valid target
  
  // private final PhotonCamera camera;
  // private PhotonPipelineResult currResult;
  // private boolean hasTargets;
  // private List<PhotonTrackedTarget> targets;
  // private PhotonTrackedTarget bestTarget;
  // private double pitch, yaw, skew, area;

  
  double xOffset, yOffset, validTarget, lastTimeStampSeconds;
  
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

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
    // currResult = camera.getLatestResult();
    // hasTargets = currResult.hasTargets();

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    jsonDumpEntry = table.getEntry("json");

    xOffset = tx.getDouble(0.0);
    yOffset = ty.getDouble(0.0);
    validTarget = tv.getDouble(0.0);
    lastTimeStampSeconds  = 0;

   tab.add("tx", tx.getDouble(0.0));
   tab.add("ty", ty.getDouble(0.0));
   tab.add("ta- area", ta.getDouble(0.0));
   tab.add("tl latency", tl.getDouble(0.0));
   tab.add("tv- boolean target", tv.getDouble(0.0));

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

  public Tuple2<Double> getSteeringOffset()
  {
    xOffset = tx.getDouble(0.0) + 0; //add any offsets if needed
    double timestamp = getTimestampSeconds();
    return new Tuple2<Double>(xOffset, timestamp);
  }

  public double getTimestampSeconds(){
    String jsonDump = jsonDumpEntry.getString("{}");  

    double currentTimeStampSeconds = lastTimeStampSeconds;
    // Attempts to get the time stamp for when the robot pose was calculated
    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      double tsValue = jsonNodeData.path("Results").path("ts").asDouble();
      SmartDashboard.putNumber("tsValue", tsValue);
      if (tsValue != 0) {
        // Converts from milleseconds to seconds
        currentTimeStampSeconds = tsValue / 1000;
      }
    } catch (JsonProcessingException e) {
      SmartDashboard.putString("Json Parsing Error", e.getStackTrace().toString());
      return -1;
    }
    return currentTimeStampSeconds;
  }

  public boolean hasTarget() 
  {
    if(tv.getDouble(0.0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  public double getTargetID(){
    return table.getEntry("tid").getDouble(-1);
  }

  // private void updateValues(){
  //   currResult = camera.getLatestResult();
  //   hasTargets = currResult.hasTargets();
  //   targets = currResult.getTargets();
  //   bestTarget = currResult.getBestTarget();
  //   yaw = bestTarget.getYaw();
  //   pitch = bestTarget.getPitch();
  //   skew = bestTarget.getSkew();
  //   area = bestTarget.getArea();
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("testtx ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    xOffset = tx.getDouble(0.0);
    validTarget = tv.getDouble(0.0);

    SmartDashboard.putNumber("xOffset", xOffset);
    SmartDashboard.putNumber("validTarget", validTarget);
        
    // updateValues();
    // SmartDashboard.putNumber("pitch", pitch);
    // SmartDashboard.putNumber("yaw", yaw);
  }

  public static synchronized VisionSubsystem getInstance(){
    if(INSTANCE == null)
      INSTANCE = new VisionSubsystem();
    return INSTANCE;
  }
}