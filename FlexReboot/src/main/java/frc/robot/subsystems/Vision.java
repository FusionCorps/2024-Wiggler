package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
      tx = () -> LimelightHelpers.getTX(llName);
      ty = () -> LimelightHelpers.getTY(llName);

      // 3d data
      this.megaTagYawGetter = yawGetter;
      LimelightHelpers.setCameraPose_RobotSpace(
          llName,
          robotToCam.getX(),
          robotToCam.getY(),
          robotToCam.getZ(),
          Units.radiansToDegrees(robotToCam.getRotation().getX()),
          Units.radiansToDegrees(robotToCam.getRotation().getY()),
          Units.radiansToDegrees(robotToCam.getRotation().getZ()));

      limelightPoseEstimate = () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
    }
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      LimelightHelpers.SetRobotOrientation(llName, megaTagYawGetter.getAsDouble(), 0, 0, 0, 0, 0);
    }
  }

  @Override
  public void simulationPeriodic() {
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
      return LimelightHelpers.getTV(llName);
    } else {
      return simCamResult.hasTargets();
    }
  }
}
