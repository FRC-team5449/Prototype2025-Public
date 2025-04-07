// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.apriltagvision;

import com.team5449.frc2025.RobotState;
import com.team5449.frc2025.RobotState.VisionObservation;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.lib.thirdpartylibs.LimelightHelpers;
import com.team5449.lib.thirdpartylibs.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  public static boolean isAligning = false;
  // Tuning constants
  private static final double MIN_TAG_AREA = 0.1;
  private static final double MAX_TAG_DISTANCE = 1.0;
  private static final double XY_STD_DEV_COEFFICIENT = 5;
  private static final double THETA_STD_DEV_COEFFICIENT = 300;
  private static final double ALIGN_THETA_STD_DEV_COEFFICIENT = 5;
  private static final double MIN_TAG_SPACING = 1.0;
  private static final double MAX_TAG_TO_CAM_DISTANCE = 2;

  public record CameraConfig(
      String limelightName, Transform3d robotToCamera, double stdDevCoefficient) {}

  private final Map<String, CameraConfig> cameras;
  private final Set<String> limelightNames;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(CameraConfig[] cameras) {
    this.cameras = new HashMap<>();
    for (CameraConfig camera : cameras) {
      this.cameras.put(camera.limelightName(), camera);
    }
    limelightNames = this.cameras.keySet();
  }

  /** Updates robot orientation for all cameras */
  public void updateRobotOrientation() {
    for (String limelightName : limelightNames) {
      LimelightHelpers.SetRobotOrientation(
          limelightName,
          Drive.gyroRotation.getDegrees(),
          Units.radiansToDegrees(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond),
          0,
          0,
          0,
          0);
    }
  }

  private Optional<VisionObservation> getMegaTag2Estimate(
      String cameraName, double stdDevCoefficient) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    Logger.recordOutput(
        "Drive/judge",
        poseEstimate == null
            || poseEstimate.tagCount == 0
            || Math.abs(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond) > 4);
    Logger.recordOutput(
        "Drive/subjudge",
        Math.abs(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond) > 4);
    if (poseEstimate == null
        || poseEstimate.tagCount == 0
        || Math.abs(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond) > 4) {
      return Optional.empty();
    }

    return processEstimate(poseEstimate, stdDevCoefficient);
  }

  @SuppressWarnings("unused")
  private Optional<VisionObservation> getMegaTagEstimate(
      String cameraName, double stdDevCoefficient) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      return Optional.empty();
    }

    // Reject single-tag measurements with high ambiguity
    if ((poseEstimate.tagCount == 1
            && poseEstimate.rawFiducials.length == 1
            && (poseEstimate.rawFiducials[0].ambiguity > 0.7
                || poseEstimate.rawFiducials[0].distToCamera > MAX_TAG_TO_CAM_DISTANCE))
        || Units.radiansToDegrees(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond)
            > 240) {
      return Optional.empty();
    }

    return processEstimate(poseEstimate, stdDevCoefficient);
  }

  private Optional<VisionObservation> processEstimate(
      PoseEstimate estimate, double stdDevCoefficient) {

    // Validate pose is within field bounds
    // if (!isOnField(estimate.pose)) {
    //   return Optional.empty();
    // }
    // Pose2d projectedPose = projectOnField(estimate.pose);
    // Calculate standard deviations
    Matrix<N3, N1> stdDevs = calculateStdDevs(estimate, stdDevCoefficient);

    return Optional.of(
        new VisionObservation(
            /*new Pose2d(estimate.pose.getTranslation(), new Rotation2d())*/ estimate.pose,
            estimate.timestampSeconds,
            stdDevs));

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
    double thetaStdDev =
        stdDevCoefficient
            * Math.pow(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond, 4)
            * (isAligning ? ALIGN_THETA_STD_DEV_COEFFICIENT : THETA_STD_DEV_COEFFICIENT);

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

  @Override
  public void periodic() {
    List<VisionObservation> observations = new ArrayList<>();

    // Update orientation for all cameras
    // updateRobotOrientation();

    // Get estimates from each camera
    for (CameraConfig camera : cameras.values()) {
      // Try MegaTag2 first
      // Optional<VisionObservation> megaTag2Estimate =
      //     getMegaTag2Estimate(camera.limelightName(), camera.stdDevCoefficient());
      // if (megaTag2Estimate.isPresent()) {
      //   observations.add(megaTag2Estimate.get());
      //   Logger.recordOutput("Vision/Timestamp", megaTag2Estimate.get().timestamp());
      //   continue;
      // }

      // Fallback to original MegaTag
      Optional<VisionObservation> megaTag2Estimate =
          getMegaTag2Estimate(camera.limelightName(), camera.stdDevCoefficient());
      megaTag2Estimate.ifPresent(observations::add);
    }

    observations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
  }
}
