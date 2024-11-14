package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final String llName = "limelight";
  NetworkTable llTable = NetworkTableInstance.getDefault().getTable(llName);

  // TODO: find camera pose relative to robot empirically
  private Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0.5), new Rotation3d());
  private AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  private Supplier<Pose2d>
      simBotPoseSupplier; // in meters, should come from drivetrain pose estimator
  private DoubleSupplier megaTagYawGetter; // in degrees, should come from drivetrain pose estimator

  private PhotonPoseEstimator photonVisionPoseEstimator;
  private PhotonPipelineResult simCamResult;
  private Supplier<LimelightHelpers.PoseEstimate> limelightPoseEstimate;

  private DoubleSupplier tx, ty;

  // use yaw getter for limelight, simbotpose for photonvision
  public Vision(DoubleSupplier yawGetter, Supplier<Pose2d> simBotPoseSupplier) {
    if (Robot.isSimulation()) {
      // set up photonvision simulation
      visionSim = new VisionSystemSim("visionSim");
      visionSim.addAprilTags(field);

      SimCameraProperties props = new SimCameraProperties();
      props.setFPS(30);
      props.setAvgLatencyMs(35);
      props.setLatencyStdDevMs(5);
      props.setCalibration(640, 480, Rotation2d.fromDegrees(100));

      cameraSim = new PhotonCameraSim(new PhotonCamera("cam0"), props);

      visionSim.addCamera(cameraSim, robotToCam);

      cameraSim.enableRawStream(true); // http://localhost:1181/
      cameraSim.enableProcessedStream(true); // http://localhost:1182/

      // Enable drawing a wireframe visualization of the field to the camera streams.
      cameraSim.enableDrawWireframe(true);

      photonVisionPoseEstimator =
          new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
      photonVisionPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.simBotPoseSupplier = simBotPoseSupplier;
    } else {
      // set up limelight
      // 2d data
      tx = () -> llTable.getEntry("tx").getDouble(0.0);
      ty = () -> llTable.getEntry("ty").getDouble(0.0);

      // 3d data
      this.megaTagYawGetter = yawGetter;

      // sets the camera's pose in the coordinate system of the robot.
      llTable
          .getEntry("camerapose_robotspace_set")
          .setDoubleArray(
              new double[] {
                robotToCam.getX(),
                robotToCam.getY(),
                robotToCam.getZ(),
                Units.radiansToDegrees(robotToCam.getRotation().getX()),
                Units.radiansToDegrees(robotToCam.getRotation().getY()),
                Units.radiansToDegrees(robotToCam.getRotation().getZ())
              });

      limelightPoseEstimate = () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
    }
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      // periodically set Robot Orientation and angular velocities in degrees and degrees per
      // second[yaw,yawrate,pitch,pitchrate,roll,rollrate]
      llTable
          .getEntry("robot_orientation_set")
          .setDoubleArray(new double[] {megaTagYawGetter.getAsDouble(), 0, 0, 0, 0, 0});
    }
  }

  @Override
  public void simulationPeriodic() {
    // only relevant in simulation with photonvision
    var simCamResults = cameraSim.getCamera().getAllUnreadResults();
    if (simCamResults != null && simCamResults.size() > 0) {
      simCamResult = simCamResults.get(simCamResults.size() - 1);
    }
    if (simBotPoseSupplier != null) visionSim.update(simBotPoseSupplier.get());
  }

  /**
   * @link
   *     https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
   * @param tagIds
   */
  public void setPreferredLLTags(int... tagIds) {
    if (Robot.isReal()) {
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", tagIds);
    }
  }

  public LimelightHelpers.PoseEstimate getLimelightPoseEstimate() {
    return limelightPoseEstimate.get();
  }

  public Optional<EstimatedRobotPose> getSimulatedPoseEstimate() {
    if (simCamResult == null) {
      return Optional.empty();
    } else {
      return photonVisionPoseEstimator.update(simCamResult);
    }
  }

  public double getTx() {
    if (Robot.isReal()) {
      return tx.getAsDouble();
    } else {
      return simCamResult.getBestTarget().getYaw();
    }
  }

  public double getTy() {
    if (Robot.isReal()) {
      return ty.getAsDouble();
    } else {
      return simCamResult.getBestTarget().getPitch();
    }
  }

  public boolean isTargetVisible() {
    if (Robot.isReal()) {
      return 1 == llTable.getEntry("tv").getDouble(0);
    } else {
      return simCamResult.hasTargets();
    }
  }

  public Vector<N3> getStdDevs() {
    if (Robot.isReal()) {
      // MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z,
      // MT2roll, MT2pitch, MT2yaw]
      double[] stdDevsRaw = llTable.getEntry("stddevs").getDoubleArray(new double[0]);
      return VecBuilder.fill(stdDevsRaw[6], stdDevsRaw[8], stdDevsRaw[11]);
    } else {
      return VecBuilder.fill(.7, .7, 999999999);
    }
  }
}
