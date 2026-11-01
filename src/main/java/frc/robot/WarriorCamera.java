// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import java.util.EnumMap;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.Data;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.measure.*;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.struct.Struct;
import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.smartdashboard.Field2d;
import frc.robot.Dashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Wraps a PhotonVision camera with pose estimation, acceptance checks, and diagnostics. */
public class WarriorCamera implements TelemetryLoggable {

  private final PhotonCamera camera;
  private Pose2d cameraPose2d = new Pose2d();
  private List<PhotonPipelineResult> cameraResult;
  private PhotonPipelineResult latestCameraResult = new PhotonPipelineResult();
  private final PhotonPoseEstimator poseEstimator;
  Optional<EstimatedRobotPose> cameraEstimatedPose;
  private Matrix<N3, N1> standardDeviation;
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  // Simulation
  private PhotonCameraSim cameraSim;
  private final SimCameraProperties simProp = new SimCameraProperties();

  public static final class CameraConstants {
    // side cam single mount
    // X: 11.868in
    // Y: 10.2795in
    // Z: 6.543in
    // Theta:

    // forward facing
    // X: 13.486in
    // Y: 9.374in
    // Z: 6.543in
    // Theta:

    // side cam dual mount
    // X: 9.6175in
    // Y: 10.5465in
    // Z: 6.443in
    // -.24, -.27, .10

    /*The amount each camera is off from the center of the robot */
    public static final Transform3d BACK_CAM_OFFSET =
        new Transform3d(
            new Translation3d(-.3425, .267, .19), // 1
            new Rotation3d(
                Units.degreesToRadians(0.5),
                Units.degreesToRadians(-24.5),
                Units.degreesToRadians(180)));
    public static final Transform3d LEFT_CAM_OFFSET =
        new Transform3d(
            new Translation3d(-.244, .2678, 0.022), // 2
            new Rotation3d(
                Units.degreesToRadians(-0.6),
                Units.degreesToRadians(-25.2),
                Units.degreesToRadians(91.0)));
    public static final Transform3d RIGHT_CAM_OFFSET =
        new Transform3d(
            new Translation3d(-.301, -.261, 0.15), // 3
            new Rotation3d(
                Units.degreesToRadians(0.5),
                Units.degreesToRadians(-22.8),
                Units.degreesToRadians(273)));

    private static final Field TAG_LAYOUT =
        Field.loadField(Fields.FRC_2026_REBUILT_ANDY_MARK);

    // Field bounds, tag distances in meters, and ambiguity thresholds for accepting a pose.
    private static final double MAXIMUM_X_POSE = TAG_LAYOUT.getFieldLength();
    private static final double MAXIMUM_Y_POSE = TAG_LAYOUT.getFieldWidth();
    private static final double APRILTAG_LIMIT_METERS = 6.7;
    private static final double APRILTAG_LIMIT_METERS_AUTO = 1.0;
    private static final double APRILTAG_CLOSE_LIMIT_METERS = 0.0;
    private static final double MAXIMUM_AMBIGUITY = 0.20;
    private static final int[] BLACKLISTED_TAG_ID_LIST = {};
    private static final int MINIMUM_ACCEPTED_TAGS = 1;

    /*
     * Standard Deviation constants, tells the robot how much to trust this data
     */
    private static final Matrix<N3, N1> LOW_SD =
        VecBuilder.fill(
            0.00001, 0.00001, Units.degreesToRadians(0.00001)); // Units of meters and radians
    private static final Matrix<N3, N1> HIGH_SD =
        VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(0.9)); // Units of meters and radians

    /** Configured alliance-specific pose offsets for field calibration. */
    public static final Transform2d ODOMETRY_BLUE_OFFSET =
        new Transform2d(Inches.of(0.0).magnitude(), Inches.of(0.0).magnitude(), new Rotation2d());

    public static final Transform2d ODOMETRY_RED_OFFSET =
        new Transform2d(Inches.of(0.0).magnitude(), Inches.of(0.0).magnitude(), new Rotation2d());
  }

  /**
   * Structured pose and reliability topics published separately from Sendable dashboard properties.
   */
  private final NetworkTable cameraStateTable;

  private final StructPublisher<Pose3d> cameraPose;
  private final StructArrayPublisher<Pose3d> targetsPose;
  private final BooleanPublisher cameraReliable;

  /** Rejection categories counted for camera diagnostics. */
  private enum PoseInvalidCause {
    OUT_OF_FIELD,
    BLACKLISTED,
    TAG_DISTANCE,
    AMBIGUITY,
    NO_TARGETS
  }

  /** Running counts by rejection category; published during the next refresh. */
  private EnumMap<PoseInvalidCause, Integer> invalidCounter = new EnumMap<>(PoseInvalidCause.class);

  /** Creates the named camera using its measured robot-to-camera transform. */
  public WarriorCamera(String cameraName, Transform3d offsets) {
    camera = new PhotonCamera(cameraName);

    // Keep diagnostics under a separate NetworkTables namespace for each camera.
    cameraStateTable = inst.getTable(camera.getName() + "CameraState");
    cameraPose = cameraStateTable.getStructTopic("Pose", Pose3d.struct).publish();
    targetsPose = cameraStateTable.getStructArrayTopic("Targets Pose", Pose3d.struct).publish();
    cameraReliable = cameraStateTable.getBooleanTopic("Reliable").publish();

    for (PoseInvalidCause cause : PoseInvalidCause.values()) {
      invalidCounter.put(cause, 0);
    }
    /*Simulation */
    simProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
    simProp.setCalibError(1.1, .08);
    simProp.setFPS(20);
    simProp.setAvgLatencyMs(35);
    simProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, simProp);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

    // Combine known field tag positions with the camera's mounting transform.
    poseEstimator = new PhotonPoseEstimator(CameraConstants.TAG_LAYOUT, offsets);

    refreshData();

    Dashboard.publish(camera.getName(), this);
  }

  /**
   * Consumes unread frames and estimates the newest pose. Call once per robot cycle. With no new
   * frame, the previously cached result and pose remain available.
   */
  public final void refreshData() {
    cameraResult = camera.getAllUnreadResults();
    if (!cameraResult.isEmpty()) {
      latestCameraResult = cameraResult.get(cameraResult.size() - 1);
      cameraEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(latestCameraResult);
      if (!cameraEstimatedPose.isEmpty()) {
        cameraPose2d = cameraEstimatedPose.get().estimatedPose.toPose2d();
        // Adds the pose to NetworkTables
        cameraPose.accept(cameraEstimatedPose.get().estimatedPose);
        setTargetsPoseList();
      }
    }
    refreshInvalid();
  }

  public boolean hasTarget() {
    return latestCameraResult.hasTargets();
  }

  /** Checks for blacklisted tag IDs and records each match; this does not test pose ambiguity. */
  public boolean isAmbiguousTags() {
    boolean containsAmbiguous = false;
    if (latestCameraResult.hasTargets()) {
      for (int c = 0; c < latestCameraResult.getTargets().size(); c++) {
        for (int i = 0; i < CameraConstants.BLACKLISTED_TAG_ID_LIST.length; i++) {
          if (latestCameraResult.getTargets().get(c).fiducialId
              == CameraConstants.BLACKLISTED_TAG_ID_LIST[i]) {
            containsAmbiguous = true;
            recordInvalid(PoseInvalidCause.BLACKLISTED);
          }
        }
        ;
      }
    }
    return containsAmbiguous;
  }

  private Distance getCameraDistance(Translation2d targetTranslation) {
    if (cameraEstimatedPose.isPresent()) {
      return Meters.of(
          cameraEstimatedPose
              .get()
              .estimatedPose
              .toPose2d()
              .getTranslation()
              .getDistance(targetTranslation));
    } else {
      return Meters.of(cameraPose2d.getTranslation().getDistance(targetTranslation));
    }
  }

  public PhotonCameraSim getSimCam() {
    return cameraSim;
  }

  public Pose2d getPose2d() {
    return cameraPose2d;
  }

  public Matrix<N3, N1> getStandardDeviation() {
    return standardDeviation;
  }

  /** Returns the cached frame's FPGA timestamp in seconds for latency-aware odometry. */
  public double getTimer() {
    return latestCameraResult.getTimestampSeconds();
  }

  public Field getTagFieldLayout() {
    return CameraConstants.TAG_LAYOUT;
  }

  /** Returns the best target's ID; callers must first check hasTarget(). */
  public int getBestTagId() {
    return latestCameraResult.getBestTarget().fiducialId;
  }

  public boolean isCameraConnected() {
    return camera.isConnected();
  }

  public double getAmbiguity() {
    if (latestCameraResult.hasTargets()) {
      return latestCameraResult.getBestTarget().poseAmbiguity;
    }
    return 1.0;
  }

  /** Returns best-target pitch in degrees; callers must first check hasTarget(). */
  public double getPitch() {
    return latestCameraResult.getBestTarget().getPitch();
  }

  /** Returns best-target yaw in degrees; callers must first check hasTarget(). */
  public double getYaw() {
    return latestCameraResult.getBestTarget().getYaw();
  }

  public int getNumberOfTags() {
    if (!cameraEstimatedPose.isEmpty()) {
      return cameraEstimatedPose.get().targetsUsed.size();
    } else {
      return 0;
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return latestCameraResult;
  }

  /** Publishes known field poses for visible tags, omitting IDs absent from the field layout. */
  public void setTargetsPoseList() {
    List<PhotonTrackedTarget> targets = latestCameraResult.getTargets();
    Pose3d[] poses =
        targets.stream()
            .map(
                target ->
                    CameraConstants.TAG_LAYOUT.getTagPose(target.getFiducialId()).orElse(null))
            .filter(p -> p != null)
            .toArray(Pose3d[]::new);
    // Publish target poses
    targetsPose.set(poses);
  }

  /**
   * Validates the cached pose against connection, field bounds, and tag-quality checks.
   *
   * @param secondRun true to reuse the current frame instead of consuming unread results again
   * @return whether the pose passes the configured checks
   */
  public boolean filterOdometry(boolean secondRun) {
    if (!secondRun) {
      refreshData();
    }

    /*Is the camera connected, or do we have a pose? */
    if (!camera.isConnected()) {
      cameraReliable.set(false);
      return false;
    } else if (!(cameraEstimatedPose.isPresent())) {
      cameraReliable.set(false);
      return false;
    }

    /*Pose in Field? */
    if (cameraPose2d.getX() > CameraConstants.MAXIMUM_X_POSE
        || cameraPose2d.getY() > CameraConstants.MAXIMUM_Y_POSE
        || cameraPose2d.getX() < 0
        || cameraPose2d.getY() < 0) {
      recordInvalid(PoseInvalidCause.OUT_OF_FIELD);
      cameraReliable.set(false);
      return false;
    }

    /*Are the tags used reliable and is there enough tags? */
    if (isTagReliable() && getNumberOfTags() >= CameraConstants.MINIMUM_ACCEPTED_TAGS) {
      standardDeviation = CameraConstants.LOW_SD;
    } else {
      // Rejected poses retain the high-noise diagnostic setting but are not fused by the
      // drivetrain.
      standardDeviation = CameraConstants.HIGH_SD;
      cameraReliable.set(false);
      return false;
    }

    /*Apply alliance offsets if there are any */
    Optional<Alliance> allianceColor = org.wpilib.driverstation.MatchState.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.RED)) {
        cameraPose2d.plus(CameraConstants.ODOMETRY_RED_OFFSET);
      } else {
        cameraPose2d.plus(CameraConstants.ODOMETRY_BLUE_OFFSET);
      }
    }

    cameraReliable.set(true);
    return true;
  }

  /**
   * Checks the distance from robot to tags and ambiguity of the tags to determine trust level of
   * data
   *
   * @return true if all conditions pass, otherwise false
   */
  public boolean isTagReliable() {

    if (latestCameraResult.hasTargets()) {
      PhotonTrackedTarget bestTarget = latestCameraResult.getBestTarget();
      int targetID = bestTarget.getFiducialId();
      Translation2d cameraTranslation2d = cameraPose2d.getTranslation();
      Translation2d targetTranslation2d =
          CameraConstants.TAG_LAYOUT.getTagPose(targetID).get().getTranslation().toTranslation2d();

      // if (DriverStation.isAutonomous()) {
      //   if (cameraTranslation2d.getDistance(targetTranslation2d) <
      // CameraConstants.APRILTAG_LIMIT_METERS_AUTO &&
      //       bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY &&
      // !isAmbiguousTags()) {
      //     return true;
      //   } else {
      //     return false;
      //   }
      // }

      // Record the first rejection reason selected by these checks.
      PoseInvalidCause invalidCause = null;
      if (cameraTranslation2d.getDistance(targetTranslation2d)
              > CameraConstants.APRILTAG_LIMIT_METERS
          || cameraTranslation2d.getDistance(targetTranslation2d)
              < CameraConstants.APRILTAG_CLOSE_LIMIT_METERS) {
        invalidCause = PoseInvalidCause.TAG_DISTANCE;
      } else if (bestTarget.getPoseAmbiguity() > CameraConstants.MAXIMUM_AMBIGUITY
          && isAmbiguousTags()) {
        invalidCause = PoseInvalidCause.AMBIGUITY;
      }

      // invalidCause being null implies there are no errors
      if (invalidCause != null) {
        cameraReliable.set(false);
        recordInvalid(invalidCause);
        return false;
      }
      cameraReliable.set(true);
      return true;
      // if (cameraTranslation2d.getDistance(targetTranslation2d) <
      // CameraConstants.APRILTAG_LIMIT_METERS
      //     && cameraTranslation2d.getDistance(targetTranslation2d) >
      // CameraConstants.APRILTAG_CLOSE_LIMIT_METERS
      //     && bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY &&
      // !isAmbiguousTags()) {
      //   cameraReliable.set(true);
      //   return true;
      // } else {
      //   cameraReliable.set(false);
      //   recordInvalid(PoseInvalidCause.UNRELIABLE);
      //   return false;
      // }
    }
    cameraReliable.set(false);
    recordInvalid(PoseInvalidCause.NO_TARGETS);
    return false;
  }

  /**
   * Increments the diagnostic counter for the reported rejection category.
   *
   * @param cause category reported by the acceptance checks
   */
  private void recordInvalid(PoseInvalidCause cause) {
    invalidCounter.put(cause, invalidCounter.get(cause) + 1);
  }

  /**
   * Update network tables with the new values of the errors logged, cycling through each unique
   * cause inside the enum
   */
  private void refreshInvalid() {
    for (PoseInvalidCause cause : PoseInvalidCause.values()) {
      cameraStateTable.getEntry(cause.name()).setInteger(invalidCounter.get(cause));
    }
  }

  @Override
  public void logTo(TelemetryTable table) {
    table.log(camera.getName() + " has 2 targets?", latestCameraResult.getTargets().size() >= 2);
    table.log(camera.getName() + " Ambiguity", getAmbiguity());
  }
}
