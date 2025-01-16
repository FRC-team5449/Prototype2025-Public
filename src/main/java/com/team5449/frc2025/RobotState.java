// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState mIntance = null;
  private Pose2d estimatedPose = new Pose2d();
  private Optional<Pose2d> poseResetRequest = Optional.empty();

  public static RobotState getInstance() {
    if (mIntance == null) {
      mIntance = new RobotState();
    }
    return mIntance;
  }

  @AutoLogOutput(key = "RobotState/Robot")
  public synchronized Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public synchronized void updatePose(Pose2d pose) {
    estimatedPose = pose;
  }

  public synchronized void requestPoseReset(Pose2d newPose) {
    poseResetRequest = Optional.of(newPose);
  }

  public synchronized Optional<Pose2d> consumePoseResetRequest() {
    Optional<Pose2d> request = poseResetRequest;
    poseResetRequest = Optional.empty();
    return request;
  }
}
