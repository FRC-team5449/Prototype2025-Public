// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.auto;

import com.team5449.frc2025.FieldConstants;
import com.team5449.frc2025.commands.IDrive;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.lib.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class AlignCommands {
  private final Drive drive;

  private Command findAndDriveToTarget(Consumer<AtomicBoolean> targetFinder) {
    IDrive iDrive = new IDrive(drive);
    AtomicBoolean targetPoseExist = new AtomicBoolean(false);
    return Commands.runOnce(() -> targetFinder.accept(targetPoseExist))
        .andThen(Commands.either(iDrive, Commands.none(), () -> targetPoseExist.get()))
        .until(() -> !targetPoseExist.get() || iDrive.atGoal());
  }

  /** If it doesn't work as before, roll back to the previous version */
  public Command driveToBranchTarget(boolean useLeftBranch, BooleanSupplier useLevel4) {
    return findAndDriveToTarget(
        (targetPoseExist) -> {
          Optional<Pose2d> tagPose =
              FieldConstants.getReefTagPose(drive.getPose().getTranslation());

          if (tagPose.isEmpty()) {
            drive.setTargetPose(null);
            targetPoseExist.set(false);
            return;
          }

          targetPoseExist.set(true);

          Transform2d transformer =
              useLeftBranch
                  ? useLevel4.getAsBoolean()
                      ? FieldConstants.leftBranchTargetPoseRelativeToTagL4
                      : FieldConstants.leftBranchTargetPoseRelativeToTag
                  : useLevel4.getAsBoolean()
                      ? FieldConstants.rightBranchTargetPoseRelativeToTagL4
                      : FieldConstants.rightBranchTargetPoseRelativeToTag;

          Pose2d branchPose =
              tagPose
                  .get()
                  .transformBy(transformer)
                  .transformBy(new Transform2d(0, 0, Rotation2d.k180deg));

          Logger.recordOutput("Odometry/BLUE TARGET", AllianceFlipUtil.apply(branchPose));
          drive.setTargetPose(branchPose);
        });
  }
}
