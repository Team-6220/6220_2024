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
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    double tx = table.getEntry("tx").getDouble(0.0);
         System.out.println(tx);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
