// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.apriltagvision;

import static edu.wpi.first.units.Units.Meters;

import com.team5449.frc2025.Constants;
import com.team5449.frc2025.RobotState;
import com.team5449.frc2025.RobotState.VisionObservation;
import com.team5449.lib.thirdpartylibs.LimelightHelpers;
import com.team5449.lib.thirdpartylibs.LimelightHelpers.PoseEstimate;
import com.team5449.lib.util.AllianceFlipUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private static final double FIELD_LENGTH_METERS = AllianceFlipUtil.fieldLength;
  private static final double FIELD_WIDTH_METERS = AllianceFlipUtil.fieldWidth;
  private static final double BOT_RADIUS =
      Math.hypot(Constants.botLength.in(Meters), Constants.botWidth.in(Meters)) / 2;
  // Tuning constants
  private static final double MIN_TAG_AREA = 0.1;
  private static final double MAX_TAG_DISTANCE = 6.0;
  private static final double XY_STD_DEV_COEFFICIENT = 0.05;
  private static final double THETA_STD_DEV_COEFFICIENT = 0.1;
  private static final double MIN_TAG_SPACING = 1.0;
  private static final double MAX_TAG_TO_CAM_DISTANCE = 3;

  public record CameraConfig(
      String limelightName, Transform3d robotToCamera, double stdDevCoefficient) {}

  private final Map<String, CameraConfig> cameras;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(CameraConfig[] cameras) {
    this.cameras = new HashMap<>();
    for (CameraConfig camera : cameras) {
      this.cameras.put(camera.limelightName(), camera);
      // Configure each camera
      LimelightHelpers.setCameraPose_RobotSpace(
          camera.limelightName(),
          camera.robotToCamera().getX(),
          camera.robotToCamera().getY(),
          camera.robotToCamera().getZ(),
          camera.robotToCamera().getRotation().getX(),
          camera.robotToCamera().getRotation().getY(),
          camera.robotToCamera().getRotation().getZ());
    }
  }

  /** Updates robot orientation for all cameras */
  public void updateRobotOrientation() {
    for (String limelightName : cameras.keySet()) {
      LimelightHelpers.SetRobotOrientation(
          limelightName, RobotState.getInstance().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }
  }

  private Optional<VisionObservation> getMegaTag2Estimate(
      String cameraName, double stdDevCoefficient) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    if (poseEstimate == null
        || poseEstimate.tagCount == 0
        || Units.radiansToDegrees(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond)
            > 720) {
      return Optional.empty();
    }

    return processEstimate(poseEstimate, stdDevCoefficient);
  }

  private Optional<VisionObservation> getMegaTagEstimate(
      String cameraName, double stdDevCoefficient) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      return Optional.empty();
    }

    // Reject single-tag measurements with high ambiguity
    if (poseEstimate.tagCount == 1
        && poseEstimate.rawFiducials.length == 1
        && (poseEstimate.rawFiducials[0].ambiguity > 0.7
            || poseEstimate.rawFiducials[0].distToCamera > MAX_TAG_TO_CAM_DISTANCE)) {
      return Optional.empty();
    }

    return processEstimate(poseEstimate, stdDevCoefficient);
  }

  private Optional<VisionObservation> processEstimate(
      PoseEstimate estimate, double stdDevCoefficient) {

    // Validate pose is within field bounds
    if (!isOnField(estimate.pose)) {
      return Optional.empty();
    }
    Pose2d projectedPose = projectOnField(estimate.pose);
    // Calculate standard deviations
    Matrix<N3, N1> stdDevs = calculateStdDevs(estimate, stdDevCoefficient);

    // TODO Which timestamp second is right
    return Optional.of(new VisionObservation(projectedPose, estimate.timestampSeconds, stdDevs));

    // return Optional.of(
    //         new VisionObservation(
    //           estimate.pose,
    //           RobotController.getFPGATime() / 1e6,
    //           stdDevs));
  }

  private Matrix<N3, N1> calculateStdDevs(
      LimelightHelpers.PoseEstimate estimate, double stdDevCoefficient) {
    // Base standard deviations
    double xyStdDev = XY_STD_DEV_COEFFICIENT * stdDevCoefficient;
    double thetaStdDev = THETA_STD_DEV_COEFFICIENT * stdDevCoefficient;

    if (estimate.tagCount > 1) {
      // Multi-tag case
      double tagSpanMultiplier = Math.min(estimate.tagSpan / MIN_TAG_SPACING, 1.0);
      double distanceMultiplier = Math.max(1.0, estimate.avgTagDist / MAX_TAG_DISTANCE);

      xyStdDev *= distanceMultiplier / (estimate.tagCount * tagSpanMultiplier);
      thetaStdDev *= distanceMultiplier / (estimate.tagCount * tagSpanMultiplier);
    } else {
      // Single-tag case
      double distanceMultiplier = Math.max(1.0, estimate.avgTagDist / MAX_TAG_DISTANCE);
      double areaMultiplier = Math.max(estimate.avgTagArea / MIN_TAG_AREA, 1.0);

      xyStdDev *= distanceMultiplier / areaMultiplier;
      thetaStdDev *= distanceMultiplier / areaMultiplier;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  /**
   * @param pose
   * @return Whether the pose is on field, allowing an error of {@code margin}.
   */
  private boolean isOnField(Pose2d pose) {
    double margin = .5;
    return pose.getX() >= -margin
        && pose.getX() <= FIELD_LENGTH_METERS + margin
        && pose.getY() >= -margin
        && pose.getY() <= FIELD_WIDTH_METERS + margin;
  }

  /**
   * Projects a robot pose on the field if it is outside the field. Apply this method if {@link
   * #isOnField()} gives {@code true}, discard this measurement otherwise.
   *
   * @param pose
   * @return projected pose
   */
  private Pose2d projectOnField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    if (x > FIELD_LENGTH_METERS - BOT_RADIUS) x = FIELD_LENGTH_METERS - BOT_RADIUS;
    else if (x < BOT_RADIUS) x = BOT_RADIUS;
    if (y > FIELD_WIDTH_METERS - BOT_RADIUS) x = FIELD_WIDTH_METERS - BOT_RADIUS;
    else if (x < BOT_RADIUS) x = BOT_RADIUS;
    return new Pose2d(x, y, pose.getRotation());
  }

  @Override
  public void periodic() {
    List<VisionObservation> observations = new ArrayList<>();

    // Update orientation for all cameras
    updateRobotOrientation();

    // Get estimates from each camera
    for (CameraConfig camera : cameras.values()) {
      // Try MegaTag2 first
      Optional<VisionObservation> megaTag2Estimate =
          getMegaTag2Estimate(camera.limelightName(), camera.stdDevCoefficient());
      if (megaTag2Estimate.isPresent()) {
        observations.add(megaTag2Estimate.get());
        Logger.recordOutput("Vision/Timestamp", megaTag2Estimate.get().timestamp());
        continue;
      }

      // Fallback to original MegaTag
      Optional<VisionObservation> megaTagEstimate =
          getMegaTagEstimate(camera.limelightName(), camera.stdDevCoefficient());
      megaTagEstimate.ifPresent(observations::add);
    }

    observations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
  }
}
