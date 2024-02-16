// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.util.TunableNumber;
import frc.robot.Constants.*;

public class PhotonVisionSubsystem extends SubsystemBase {
  private static PhotonVisionSubsystem INSTANCE = null;
  /** Creates a new PhotonVisionSubsystem. */

  // private final TunableNumber photonTurn_kP = new TunableNumber("photonTurn_kP", PhotonVisionConstants.LINEAR_P);
  // private final TunableNumber photonTurn_kI = new TunableNumber("photonTurn_kI", PhotonVisionConstants.LINEAR_I);
  // private final TunableNumber photonTurn_kD = new TunableNumber("photonTurn_kD", PhotonVisionConstants.LINEAR_D);
  // private final TunableNumber photonTurn_kIZone = new TunableNumber("photonTurn_kIZone", PhotonVisionConstants.LINEAR_IZONE);
  // private final TunableNumber photonTurn_maxVel = new TunableNumber("photonTurn_maxVel", PhotonVisionConstants.LINEAR_MAXVEL);
  // private final TunableNumber photonTurn_maxAccel = new TunableNumber("photonTurn_maxAccel", PhotonVisionConstants.LINEAR_MAXIMUMACCEL);
  // private final TunableNumber photonTurn_Tolerance = new TunableNumber("photonTurn_Tolerance", PhotonVisionConstants.LINEAR_Tolerance);

  // private final TunableNumber photonTransit_kP = new TunableNumber("photonTransit_kP", PhotonVisionConstants.ANGULAR_P);
  // private final TunableNumber photonTransit_kI = new TunableNumber("photonTransit_kI", PhotonVisionConstants.ANGULAR_I);
  // private final TunableNumber photonTransit_kD = new TunableNumber("photonTransit_kD", PhotonVisionConstants.ANGULAR_D);
  // private final TunableNumber photonTransit_kIZone = new TunableNumber("photonTransit_kIZone", PhotonVisionConstants.ANGULAR_IZONE);
  // private final TunableNumber photonTransit_maxVel = new TunableNumber("photonTransit_maxVel", PhotonVisionConstants.ANGULAR_MAXVEL);
  // private final TunableNumber photonTransit_maxAccel = new TunableNumber("photonTransit_maxAccel", PhotonVisionConstants.ANGULAR_MAXIMUMACCEL);
  // private final TunableNumber photonTransit_Tolerance = new TunableNumber("photonTransit_Tolerance", PhotonVisionConstants.ANGULAR_Tolerance);

  private PhotonCamera camera = new PhotonCamera("photonvision");
  private PhotonPipelineResult currResult;
  private boolean hasTargets;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private double pitch, yaw, skew, area;

  // private final ProfiledPIDController s_turnController, s_transitionController;
  // private final TrapezoidProfile.Constraints s_turnConstraints, s_transitionConstraints;


  private PhotonVisionSubsystem() {
    currResult = camera.getLatestResult();
    hasTargets = currResult.hasTargets();


    // this.s_turnConstraints = new TrapezoidProfile.Constraints(photonTurn_maxVel.get(), photonTurn_maxAccel.get());
    // this.s_transitionConstraints = new TrapezoidProfile.Constraints(photonTransit_maxVel.get(), photonTransit_maxAccel.get());

    // this.s_turnController = new ProfiledPIDController(
    //     photonTurn_kP.get(),
    //     photonTurn_kI.get(),
    //     photonTurn_kD.get(),
    //     s_turnConstraints
    //     );

    // this.s_transitionController = new ProfiledPIDController(
    //     photonTransit_kP.get(),
    //     photonTransit_kI.get(),
    //     photonTransit_kD.get(),
    //     s_transitionConstraints
    //     );
    // //Using I only withing 3 degrees of error
    // this.s_turnController.setIZone(photonTurn_kIZone.get());
    // this.s_transitionController.setIZone(photonTransit_kIZone.get());

    // //Setting Tolerance
    // this.s_turnController.setTolerance(photonTransit_Tolerance.get());
    // this.s_turnController.setTolerance(photonTurn_Tolerance.get());
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

  public boolean getHasTargets()
  {
    return hasTargets;
  }

  public double getTurnOffset()
  {
    if(hasTargets)
    {
      SmartDashboard.putNumber("yaw", yaw);
      return yaw;
    }
    return -1.0; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateValues();
  }

  public static synchronized PhotonVisionSubsystem getInstance(){
    if(INSTANCE == null)
      INSTANCE = new PhotonVisionSubsystem();
    return INSTANCE;
  }
}
