// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.lib.util.AllianceFlipUtil;
import com.team5449.lib.util.GeomUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState mIntance = null;
  private Pose2d estimatedPose = new Pose2d();
  private Optional<Pose2d> poseResetRequest = Optional.empty();
  private Optional<Pose2d> targetResetRequest = Optional.empty();
  private Optional<VisionObservation> latestVisionObservation = Optional.empty();
  private VisionObservation lastProcessedObservation = null;
  @Getter private Rotation2d offsetAngle = new Rotation2d();

  @Getter
  @AutoLogOutput(key = "RobotState/RobotSpeeds")
  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  public static RobotState getInstance() {
    if (mIntance == null) {
      mIntance = new RobotState();
    }
    return mIntance;
  }

  @AutoLogOutput(key = "RobotState/Odometry")
  public synchronized Pose2d getPose() {
    return estimatedPose;
  }

  public synchronized void updateSpeeds(ChassisSpeeds speeds) {
    robotSpeeds = speeds;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public Twist2d fieldVelocity() {
    Twist2d robotVelocity = GeomUtil.toTwist2d(robotSpeeds);
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  public synchronized void updatePose(Pose2d pose) {
    estimatedPose = pose;
  }

  public synchronized void resetHeading() {
    setPose(
        new Pose2d(
            estimatedPose.getTranslation(),
            AllianceFlipUtil.shouldFlip() ? Rotation2d.kZero : Rotation2d.k180deg));
    offsetAngle = Drive.rawGyroRotation;
  }

  public synchronized Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public synchronized void setPose(Pose2d newPose) {
    poseResetRequest = Optional.of(newPose);
  }

  public synchronized void setTargetPose(Pose2d newPose) {
    targetResetRequest = Optional.of(newPose);
  }

  public synchronized Optional<Pose2d> consumePoseResetRequest() {
    Optional<Pose2d> request = poseResetRequest;
    poseResetRequest = Optional.empty();
    return request;
  }

  public synchronized Optional<Pose2d> consumeTargetResetRequest() {
    Optional<Pose2d> request = targetResetRequest;
    targetResetRequest = Optional.empty();
    return request;
  }

  public synchronized void addVisionObservation(VisionObservation newObservation) {

    // Check if this is a duplicate measurement
    if (lastProcessedObservation != null
        && lastProcessedObservation.timestamp() == newObservation.timestamp()
        && lastProcessedObservation.visionPose().equals(newObservation.visionPose)) {
      return; // Skip duplicate measurement
    }

    latestVisionObservation = Optional.of(newObservation);
  }

  public synchronized Optional<VisionObservation> consumeVisionObservation() {
    Optional<VisionObservation> observation = latestVisionObservation;
    if (observation.isPresent()) {
      lastProcessedObservation = observation.get();
      latestVisionObservation = Optional.empty();
    }
    return observation;
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
