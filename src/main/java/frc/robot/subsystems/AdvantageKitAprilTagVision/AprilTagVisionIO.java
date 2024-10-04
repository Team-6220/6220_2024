package frc.robot.subsystems.AdvantageKitAprilTagVision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface AprilTagVisionIO {
    class AprilTagVisionIOInputs implements LoggableInputs {
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    // public double skew = 0.0;//Not available in april tag mode
    public double[] corners = {0.0};
    public Transform2d cameraToTarget = new Transform2d();
    public int fiducialId = -1;
    public double poseAmbiguity = -1.0;
    public Transform3d bestcameratotarget = new Transform3d();
    public Transform3d alternativecameratotarget = new Transform3d();

    @Override
    public void toLog(LogTable table) {
      table.put("yaw", yaw);
      // table.put("Timestamps", timestamps);
      // table.put("FrameCount", frames.length);
      // for (int i = 0; i < frames.length; i++) {
      //   table.put("Frame/" + i, frames[i]);
      // }
      // table.put("DemoFrame", demoFrame);
      // table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      // timestamps = table.get("Timestamps", new double[] {0.0});
      // int frameCount = table.get("FrameCount", 0);
      // frames = new double[frameCount][];
      // for (int i = 0; i < frameCount; i++) {
      //   frames[i] = table.get("Frame/" + i, new double[] {});
      // }
      // demoFrame = table.get("DemoFrame", new double[] {});
      // fps = table.get("Fps", 0);
      yaw = table.get("yaw", 0.0);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {} 
}
