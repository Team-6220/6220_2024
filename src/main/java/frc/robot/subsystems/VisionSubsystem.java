// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final VisionSubsystem INSTANCE = null;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
  // Check https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api for details
  private double tx = table.getEntry("tx").getDouble(0.0); //horizontal offset
  private double ty = table.getEntry("ty").getDouble(0.0); //vertical offset
  private double ta = table.getEntry("ta").getDouble(0.0); //Target area (0% of image to 100% of image)
  private double tl = table.getEntry("tl").getDouble(0.0); //The pipeline's latency contribution (ms). Add to "cl" to get total latency.
  private boolean tv = table.getEntry("tv").getBoolean(false); //check whetehr the limelight have any valid target

  private double distanceFromLimelightToGoalInches;

  double limelightMountAngleDegrees = 0;// TODO:GET THIS INTO CONSTANT + CHANGE ACCORDINGLY
  double limelightLensHeightInches = 20;// TODO:GET THIS INTO CONSTANT + CHANGE ACCORDINGLY 

  // Check https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
  private double targetHeight = 0; //TODO: CHANGE ACCORDINGLY -- The height of the target, property of the field? (h2 in the equation in the link)
  private double heightOfCamAboveFloor = 0; //TODO: CHANGE ACCORDINGLY -- The height of your camera above the floor (h1 in the equation in the link)


  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
   tx = table.getEntry("tx").getDouble(0.0);
   ty = table.getEntry("ty").getDouble(0.0);
   ta = table.getEntry("ta").getDouble(0.0);
   tl = table.getEntry("tl").getDouble(0.0);
   tv = table.getEntry("tv").getBoolean(false);

   tab.add("tx", tx);
   tab.add("ty", ty);
   tab.add("ta- area", ta);
   tab.add("tl latency", tl);
   tab.add("tv- boolean target", tv);

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
    ty = table.getEntry("ty").getDouble(0.0);
      double verticalOffset = ty; //add anything if needed
      double angle = (verticalOffset+limelightMountAngleDegrees)*(3.14159/180);
      distanceFromLimelightToGoalInches = (targetHeight-heightOfCamAboveFloor)/Math.tan(angle);
    return distanceFromLimelightToGoalInches;
  }

  public double getSteeringOffset()
  {
    tx = table.getEntry("tx").getDouble(0.0) + 0; //add any offsets if needed
    return tx;
  }

  public static VisionSubsystem getInstance()
  {
    if (INSTANCE == null) {
      return new VisionSubsystem();
  }
  return INSTANCE;
  }
}
