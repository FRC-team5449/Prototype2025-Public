// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.apriltagvision;

import com.team5449.frc2025.FieldConstants;
import com.team5449.frc2025.RobotState;
import com.team5449.frc2025.RobotState.VisionObservation;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.lib.thirdpartylibs.LimelightHelpers;
import com.team5449.lib.thirdpartylibs.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private static final double FIELD_LENGTH_METERS = FieldConstants.fieldLength;
  private static final double FIELD_WIDTH_METERS = FieldConstants.fieldWidth;
  // Tuning constants
  private static final double MIN_TAG_AREA = 0.1;
  private static final double MAX_TAG_DISTANCE = 1.0;
  private static final double XY_STD_DEV_COEFFICIENT = 5;
  private static final double THETA_STD_DEV_COEFFICIENT = 100;
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
    if (poseEstimate == null
        || poseEstimate.tagCount == 0
        || Units.radiansToDegrees(RobotState.getInstance().getRobotSpeeds().omegaRadiansPerSecond)
            > 240) {
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
  @SuppressWarnings("unused")
  private boolean isOnField(Pose2d pose) {
    double margin = .5;
    return pose.getX() >= -margin
        && pose.getX() <= FIELD_LENGTH_METERS + margin
        && pose.getY() >= -margin
        && pose.getY() <= FIELD_WIDTH_METERS + margin;
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

  /**
   * Get the target info for a specific AprilTag
   *
   * @param cameraName The name of the limelight camera
   * @param tagId The ID of the AprilTag
   * @return The target info, or null if not visible
   */
  private LimelightHelpers.RawFiducial getTargetInfo(String cameraName) {
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(cameraName);

    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      for (int tagId : FieldConstants.tagIds) {
        if (fiducial.id == tagId) {
          return fiducial;
        }
      }
    }

    return null;
  }

  /**
   * Get the longitudinal distance (forward/backward) to an AprilTag
   *
   * @param cameraName The name of the limelight camera
   * @param tagId The ID of the AprilTag
   * @return The longitudinal distance in meters, or empty if not visible
   */
  public Optional<Double> getLongitudinalDistance(String cameraName) {
    // First check if the specific tag is visible
    LimelightHelpers.RawFiducial targetInfo = getTargetInfo(cameraName);
    if (targetInfo == null) {
      return Optional.empty();
    }

    // Get the target pose in robot space directly from Limelight
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    if (targetPose.length < 6) {
      return Optional.empty();
    }

    // The X component represents the forward/backward distance in robot space
    double longitudinalDistance = -targetPose[1];

    Logger.recordOutput("Vision/Longitudinal Dist", longitudinalDistance);

    return Optional.of(longitudinalDistance);
  }

  /**
   * Get the lateral distance (side-to-side) to an AprilTag
   *
   * @param cameraName The name of the limelight camera
   * @param tagId The ID of the AprilTag
   * @return The lateral distance in meters, or empty if not visible
   */
  public Optional<Double> getLateralDistance(String cameraName) {
    // First check if the specific tag is visible
    LimelightHelpers.RawFiducial targetInfo = getTargetInfo(cameraName);
    if (targetInfo == null) {
      return Optional.empty();
    }

    // Get the target pose in robot space directly from Limelight
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    if (targetPose.length < 6) {
      return Optional.empty();
    }

    // The Y component represents the side-to-side distance in robot space
    // Negative because right is negative in robot space
    double lateralDistance = -targetPose[0];

    Logger.recordOutput("Vision/Lateral Dist", lateralDistance);

    return Optional.of(lateralDistance);
  }

  /**
   * Get the rotation between the robot and a specific AprilTag using raw data This returns the
   * horizontal angle to the tag (how far left/right it is)
   *
   * @param cameraName The name of the limelight camera
   * @param tagId The ID of the AprilTag
   * @return The horizontal angle in degrees, or empty if not visible
   */
  public Optional<Double> getTagHorizontalAngle(String cameraName) {
    // First check if the specific tag is visible
    LimelightHelpers.RawFiducial targetInfo = getTargetInfo(cameraName);
    if (targetInfo == null) {
      return Optional.empty();
    }

    // Get the target pose in robot space directly from Limelight
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    if (targetPose.length < 6) {
      return Optional.empty();
    }

    // The yaw component (index 5) gives us the horizontal angle
    double horizontalAngle = targetPose[4];

    Logger.recordOutput("Vision/Tag Horizontal Angle", horizontalAngle);

    // Negate the angle to match your previous implementation's convention
    return Optional.of(-horizontalAngle);
  }

  public Optional<Integer> getTagId(String cameraName) {
    double targetId = LimelightHelpers.getFiducialID(cameraName);
    if ((int) targetId == -1) {
      return Optional.empty();
    }

    return Optional.of((int) targetId);
  }

  public Optional<Pose3d> getTagPoseRelativeToRobot(String cameraName) {
    // LimelightHelpers.RawFiducial targetInfo = getTargetInfo(cameraName);
    // if (targetInfo == null) {
    //   return Optional.empty();
    // }

    // Get the target pose in robot space directly from Limelight
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    Logger.recordOutput("Target Pose", targetPose);
    if (targetPose.length < 6) {
      return Optional.empty();
    }

    return Optional.of(
        new Pose3d(
            targetPose[0],
            targetPose[2],
            targetPose[1],
            new Rotation3d(targetPose[3], targetPose[4], targetPose[5])));
  }
}
