package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve;

public final class LimelightCalculations {
    public static void updatePoseEstimation(SwerveDrivePoseEstimator poseEstimator, Swerve s_Swerve) {
        LimelightResults limelightResults = LimelightHelpers.getLatestResults(VisionConstants.LIMELIGHT3_NAME_STRING);
        
        if(limelightResults.targetingResults.targets_Fiducials == null) {
            System.out.println("No Fiducial Targets");
            return;
        }

        
        Pose2d botPose2d = updateVisionRobotPose2d(limelightResults, poseEstimator, s_Swerve.visionMeasurementStdDevConstant.get());
        if(botPose2d == null) {
            return;
        }

        double latency = limelightResults.targetingResults.latency_capture + limelightResults.targetingResults.latency_jsonParse;
        
        Translation2d botTranslation2d = botPose2d.getTranslation();


        poseEstimator.addVisionMeasurement(new Pose2d(botTranslation2d, s_Swerve.getHeading()), Timer.getFPGATimestamp() - (latency/1000.0));
        System.out.println("AddedVisionMeasurement");        
    }

    public static Pose2d updateVisionRobotPose2d(LimelightResults limelightResults, SwerveDrivePoseEstimator poseEstimator, double constant) {
        int count = 0;
        double totalX = 0;
        double totalY = 0;
        double totalAngle = 0;
        double totalDistance = 0;

        for(LimelightTarget_Fiducial aprilTag : limelightResults.targetingResults.targets_Fiducials) {
            Pose2d aprilTagPoseEstimate = aprilTag.getRobotPose_FieldSpace2D();
            Pose2d currentPose = poseEstimator.getEstimatedPosition();
            if(Math.abs(aprilTagPoseEstimate.getX() - currentPose.getX()) < 1
            && Math.abs(aprilTagPoseEstimate.getY() - currentPose.getY()) < 1) {
                totalX += aprilTagPoseEstimate.getX();
                totalY += aprilTagPoseEstimate.getY();
                totalAngle += aprilTagPoseEstimate.getRotation().getRadians();
                totalDistance = aprilTag.getTargetPose_CameraSpace().getTranslation().getDistance(new Translation3d(0, 0, 0));
                count++;
            } else {
                System.out.println("Bad Vision Data");
            }
        }
        if(count > 0) {
            double visionStdDev = constant * (totalDistance/count);
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev, visionStdDev, visionStdDev));
            return new Pose2d(new Translation2d(totalX/count, totalY/count), new Rotation2d(totalAngle/count));
            
        }
        return null;
    }   
}
