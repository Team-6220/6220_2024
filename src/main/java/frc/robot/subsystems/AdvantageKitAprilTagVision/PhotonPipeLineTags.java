package frc.robot.subsystems.AdvantageKitAprilTagVision;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public record PhotonPipeLineTags
(
    double yaw,
    double pitch,
    double area,
    double skew,
    int fiducialId,
    Transform3d bestCameraToTargetpose,
    Transform3d altCameraToTargetPose,
    double poseAmbiguity,
    List<TargetCorner> minAreaRectCorners,
    List<TargetCorner> detectedCorners
)
{}
