package frc.robot.subsystems.AdvantageKitAprilTagVision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.subsystems.AdvantageKitAprilTagVision.AprilTagVisionIO;

public class AprilTagVisionIOArduCamPhoton implements AprilTagVisionIO {
      private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 10;
  private static final int cameraGain = 25;

  private double yaw;

  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  // private AprilTagLayoutType lastAprilTagType = null;

  private final StringPublisher tagLayoutPublisher;
  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber demoObservationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  PhotonCamera cam = whatsoever (name);

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();

  public AprilTagVisionIONorthstar(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, int index) {
    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    var northstarTable = NetworkTableInstance.getDefault().getTable(instanceNames[index]);
    var configTable = northstarTable.getSubTable("config");
    configTable.getStringTopic("camera_id").publish().set(cameraIds[index]);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    demoObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("demo_observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

    disconnectedAlert =
        new Alert("No data from \"" + instanceNames[index] + "\"", Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    Apriltagvisionioinputs.yaw = PhotoncamerName.getyaw;
    // Publish tag layout
    var aprilTagType = aprilTagTypeSupplier.get();
    if (aprilTagType != lastAprilTagType) {
      lastAprilTagType = aprilTagType;
      tagLayoutPublisher.set(aprilTagType.getLayoutString());
    }

    // Get observations
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.demoFrame = new double[] {};
    for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
      inputs.demoFrame = demoFrame;
    }
    inputs.fps = fpsSubscriber.get();

    // Update disconnected alert
    if (queue.length > 0) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }
}
