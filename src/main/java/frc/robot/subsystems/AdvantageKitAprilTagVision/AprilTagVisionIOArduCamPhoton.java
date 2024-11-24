// package frc.robot.subsystems.AdvantageKitAprilTagVision;

// import java.util.ArrayList;
// import java.util.Optional;
// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.IntegerSubscriber;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.PubSubOption;
// import edu.wpi.first.networktables.StringPublisher;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.AdvantageKitAprilTagVision.AprilTagVisionIO;

// public class AprilTagVisionIOArduCamPhoton implements AprilTagVisionIO {

//   private final PhotonCamera camera;
//   private final PhotonPoseEstimator poseEstimator;
//   // PhotonPipelineResult photonPipelineResult = new PhotonPipelineResult();

//   // private static final double disconnectedTimeout = 0.5;
//   // private final Alert disconnectedAlert;
//   // private final Timer disconnectedTimer = new Timer();
  


//   public AprilTagVisionIOArduCamPhoton(String cameraName, Transform3d robotToCamera) {
//     // this.aprilTagTypeSupplier = aprilTagTypeSupplier;
//     // var northstarTable = NetworkTableInstance.getDefault().getTable(instanceNames[index]);
//     // var configTable = northstarTable.getSubTable("config");
//     // configTable.getStringTopic("camera_id").publish().set(cameraIds[index]);
//     // configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
//     // configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
//     // configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
//     // configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
//     // configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
//     // configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
//     // tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();

//     // var outputTable = northstarTable.getSubTable("output");
//     // observationSubscriber =
//     //     outputTable
//     //         .getDoubleArrayTopic("observations")
//     //         .subscribe(
//     //             new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
//     // demoObservationSubscriber =
//     //     outputTable
//     //         .getDoubleArrayTopic("demo_observations")
//     //         .subscribe(
//     //             new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
//     // fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

//     // disconnectedAlert =
//     //     new Alert("No data from \"" + instanceNames[index] + "\"", Alert.AlertType.ERROR);
//     // disconnectedTimer.start();
//     camera = new PhotonCamera(cameraName);

//     camera.setPipelineIndex(0);

//     poseEstimator = new PhotonPoseEstimator(
//       VisionConstants.apriltagLayout, // ok so they have different apriltaglayouts existing in constant (used switch case to change the layout) but we only need one so em
//       PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//       camera,
//       robotToCamera
//       );
//     poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
//   }

//   // public static void updateCamerasPoseEstimation(SwerveDrivePoseEstimator swervePoseEstimator, double camTrustValues)
//   // {
    
//   // }


//   public void updateInputs(AprilTagVisionIOInputs inputs)
//   {
//     // inputs.estimate = new ArrayList<>();
    
//     // PoseStrategy.valueOf(getCameraName()).toString();

//     // double timestamp = Logger.getTimestamp(); //This is used to calculate how fast update inputs run, I don't think we need it but I'll still keep it
//     Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
//     estimatedRobotPose.ifPresent
//     (
//        estimate -> 
//        {
//          //Per april tag, not really used...
//           // int[] tagIDs = new int[estimate.targetsUsed.size()];
//           // double avgDistance = 0.0;
//           // int numTags = 0;

//           // Pose3d robotPose = estimate.estimatedPose;

//           // for (int i = 0; i < estimate.targetsUsed.size(); i++) {
//           // tagIDs[i] = estimate.targetsUsed.get(i).getFiducialId();

//           // Optional<Pose3d> tagPose = VisionConstants.apriltagLayout.getTagPose(tagIDs[i]);

//           // if (tagPose.isPresent())
//             // {
//             //   numTags++;
//             //   avgDistance +=
//             //   tagPose.get().getTranslation().getDistance(robotPose.getTranslation());
//             // }
//           // }

//           // avgDistance /= numTags;

//           inputs.estimate = estimate;
//       }
//     );
//     if(!estimatedRobotPose.isPresent())
//     {
//       inputs.estimate = new EstimatedRobotPose(new Pose3d(new Pose2d(-1,-1, new Rotation2d(0))), 0, null, null);
//     }
//   }

//   @Override
//   public String getCameraName() {
//       // TODO Auto-generated method stub
//       return camera.getName();
//   }
// }
