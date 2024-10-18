package frc.robot.subsystems.AdvantageKitAprilTagVision;

import java.util.ArrayList;
// import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public interface AprilTagVisionIO {

    void updateInputs(AprilTagVisionIOInputs inputs);

    String getCameraName();

    class AprilTagVisionIOInputs implements LoggableInputs {
      public EstimatedRobotPose estimate;
      public String cameraName;
    // public List<PhotonPipeLineTags> tags = new ArrayList<>(); //hash map?
    // PhotonTrackedTarget photontarget = new PhotonTrackedTarget(0, 0, 0, 0, 0, null, null, 0, null, null)
    // public double yaw = 0.0;
    // public double pitch = 0.0;
    // public double area = 0.0;
    // // public double skew = 0.0;//Not available in april tag mode
    // public double[] corners = {0.0};
    // public Transform2d cameraToTarget = new Transform2d();
    // public int fiducialId = -1;
    // public double poseAmbiguity = -1.0;
    // public Transform3d bestcameratotarget = new Transform3d();
    // public Transform3d alternativecameratotarget = new Transform3d(); 
    

    @Override
    public void toLog(LogTable table) {
      // table.put("NumEstimates", estimate.size());
      
      // for (int i = 0; i < estimates.size(); i++) {
        

        // table.put(tableKey + "Pose0", poseToLog(estimate.));
        // table.put(tableKey + "Pose1", poseToLog(estimate.pose1()));

        // table.put(tableKey + "Ambiguity0", estimate.ambiguity0());
        // table.put(tableKey + "Ambiguity1", estimate.ambiguity1());

        // table.put(tableKey + "AverageTagDistance0", estimate.averageTagDistance0());
        // table.put(tableKey + "AverageTagDistance1", estimate.averageTagDistance1());

        // table.put(tableKey + "Timestamp", estimate.timestamp());
        // table.put(
        //     tableKey + "TagIDs",
        //     Arrays.stream(estimate.tagIDs()).mapToLong(Long::valueOf).toArray());

      // }
      String tableKey = "Estimates/" + cameraName + "/";
    
      table.put(tableKey + "timestampSeconds" + estimate.timestampSeconds);
      table.put(tableKey + "Pose" + poseToLog(estimate.estimatedPose));
      table.put(tableKey + "Strategy" + estimate.strategy.toString());
    }

    @Override
    public void fromLog(LogTable table) {
      // int numEstimates = table.get("NumEstimates", 0);

      // for (int i = 0; i < numEstimates; i++) {
        String tableKey = "Estimates/" + cameraName + "/";

        // Pose3d pose0 = poseFromLog(table.get(tableKey + "Pose0", new double[0]));
        // double ambiguity0 = table.get(tableKey + "Ambiguity0", 0.0);
        // Pose3d pose1 = poseFromLog(table.get(tableKey + "Pose1", new double[0]));
        // double ambiguity1 = table.get(tableKey + "Ambiguity1", 0.0);
        // int[] tagIDs =
        //     Arrays.stream(table.get(tableKey + "TagIDs", new long[0]))
        //         .mapToInt(n -> (int) n)
        //         .toArray();
        // double averageTagDistance0 = table.get(tableKey + "AverageTagDistance0", 0.0);
        // double averageTagDistance1 = table.get(tableKey + "AverageTagDistance1", 0.0);
        double timestamp = table.get(tableKey + "timestampSeconds", 0.0);
        Pose3d pose = poseFromLog(table.get(tableKey + "Pose", new double[0]));
        // PoseStrategy strategy = (table.get(tableKey."Strategy", "") == "") ? : null;
        estimate = new EstimatedRobotPose(pose, timestamp, null, null);
    // }
  }
 
      private static Pose3d poseFromLog(double[] loggedPose) {
      if (loggedPose.length != 7) {
        return null;
      }

      return new Pose3d(
          new Translation3d(loggedPose[0], loggedPose[1], loggedPose[2]),
          new Rotation3d(
              new Quaternion(loggedPose[3], loggedPose[4], loggedPose[5], loggedPose[6])));
    }

    private static double[] poseToLog(Pose3d pose) {
      if (pose == null) {
        return new double[0];
      }

      return new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(),
        pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()
      };
    }
  }
}