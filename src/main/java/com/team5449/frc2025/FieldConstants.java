// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

public class FieldConstants {
  public static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final Translation2d center =
      new Translation2d(Units.inchesToMeters(176.746), tagLayout.getFieldWidth() / 2.0);

  public static final Pose2d[] blueCenterFaces =
      new Pose2d[6]; // Starting facing the driver station in clockwise order
  public static final Pose2d[] redCenterFaces = new Pose2d[6];

  public static final int[] blueTagIds = {17, 18, 19, 20, 21, 22};
  public static final int[] redTagIds = {6, 7, 8, 9, 10, 11};

  static {
    blueCenterFaces[0] = tagLayout.getTagPose(18).get().toPose2d();
    blueCenterFaces[1] = tagLayout.getTagPose(19).get().toPose2d();
    blueCenterFaces[2] = tagLayout.getTagPose(20).get().toPose2d();
    blueCenterFaces[3] = tagLayout.getTagPose(21).get().toPose2d();
    blueCenterFaces[4] = tagLayout.getTagPose(22).get().toPose2d();
    blueCenterFaces[5] = tagLayout.getTagPose(17).get().toPose2d();

    redCenterFaces[0] = tagLayout.getTagPose(7).get().toPose2d();
    redCenterFaces[1] = tagLayout.getTagPose(8).get().toPose2d();
    redCenterFaces[2] = tagLayout.getTagPose(9).get().toPose2d();
    redCenterFaces[3] = tagLayout.getTagPose(10).get().toPose2d();
    redCenterFaces[4] = tagLayout.getTagPose(11).get().toPose2d();
    redCenterFaces[5] = tagLayout.getTagPose(6).get().toPose2d();
  }

  public static final List<Pose2d> targetPoses =
      new ArrayList<Pose2d>(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? List.of(blueCenterFaces)
              : List.of(redCenterFaces));

  public static final int[] tagIds =
      DriverStation.getAlliance().get() == Alliance.Blue ? blueTagIds : redTagIds;

  public static final Transform2d leftBranchTargetPoseRelativeToTag =
      new Transform2d(1, 0, Rotation2d.kZero);
  public static final Transform2d rightBranchTargetPoseRelativeToTag =
      new Transform2d(1, 0, Rotation2d.kZero);
}
