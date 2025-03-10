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
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorSubsystem;
import com.team5449.lib.util.GeomUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
@RequiredArgsConstructor
public class AutoCommand {
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final EndEffectorSubsystem endEffector;
  private final AprilTagVision vision;

  /**
   * Creates a command that aligns the robot with an AprilTag using ProfiledPIDControllers.
   *
   * @param tagId The ID of the AprilTag to align with
   * @param cameraName The name of the camera to use for vision
   * @param targetLongitudinalDistance The desired forward/backward distance to maintain from the
   *     tag (meters)
   * @param targetLateralDistance The desired side-to-side distance to maintain from the tag
   *     (meters)
   * @return A command that aligns the robot with the specified AprilTag
   */
  public Command alignWithAprilTag(
      int tagId,
      String cameraName,
      double targetLongitudinalDistance,
      double targetLateralDistance) {

    // Create constraints for the motion profiles
    TrapezoidProfile.Constraints longitudinalConstraints =
        new TrapezoidProfile.Constraints(1.0, 0.5); // Max velocity (m/s), Max acceleration (m/s²)
    TrapezoidProfile.Constraints lateralConstraints =
        new TrapezoidProfile.Constraints(0.8, 0.4); // Max velocity (m/s), Max acceleration (m/s²)

    // Create ProfiledPIDControllers for longitudinal and lateral motion
    ProfiledPIDController longitudinalController =
        new ProfiledPIDController(0.8, 0.0, 0, longitudinalConstraints);
    ProfiledPIDController lateralController =
        new ProfiledPIDController(0.8, 0.0, 0, lateralConstraints);

    // Set tolerance for each controller
    longitudinalController.setTolerance(0.05); // 5cm tolerance
    lateralController.setTolerance(0.05); // 5cm tolerance

    Command alignCommand =
        Commands.sequence(
            // Initialize controllers
            Commands.runOnce(
                () -> {
                  System.out.println("Starting alignment with AprilTag " + tagId);
                  longitudinalController.reset(0);
                  lateralController.reset(0);
                }),

            // Main alignment command
            Commands.run(
                    () -> {
                      // Get current distances to the tag
                      Optional<Double> longDistOpt = vision.getLongitudinalDistance(cameraName);
                      Optional<Double> latDistOpt = vision.getLateralDistance(cameraName);

                      if (longDistOpt.isEmpty() || latDistOpt.isEmpty()) {
                        // Tag not visible, stop or rotate to search
                        drive.stop();
                        System.out.println("AprilTag " + tagId + " not visible");
                        return;
                      }

                      // Calculate errors
                      double longitudinalError = longDistOpt.get() - targetLongitudinalDistance;
                      double lateralError = latDistOpt.get() - targetLateralDistance;

                      // Calculate drive speeds using ProfiledPIDControllers
                      double forwardSpeed = -longitudinalController.calculate(longitudinalError, 0);
                      double strafeSpeed = -lateralController.calculate(lateralError, 0);

                      // Apply drive outputs with deadband
                      if (Math.abs(forwardSpeed) < 0.05 && longitudinalController.atGoal()) {
                        forwardSpeed = 0;
                      }

                      if (Math.abs(strafeSpeed) < 0.05 && lateralController.atGoal()) {
                        strafeSpeed = 0;
                      }

                      // Create robot-relative ChassisSpeeds
                      ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, 0);
                      drive.runVelocity(speeds);
                    })
                .until(
                    () -> {
                      // Check if we're within tolerance
                      Optional<Double> longDistOpt = vision.getLongitudinalDistance(cameraName);
                      Optional<Double> latDistOpt = vision.getLateralDistance(cameraName);

                      if (longDistOpt.isEmpty() || latDistOpt.isEmpty()) {
                        return false; // Can't end if we don't see the tag
                      }

                      double longitudinalError = longDistOpt.get() - targetLongitudinalDistance;
                      double lateralError = latDistOpt.get() - targetLateralDistance;

                      // Update controllers to check if at goal
                      longitudinalController.calculate(longitudinalError, 0);
                      lateralController.calculate(lateralError, 0);

                      return longitudinalController.atGoal() && lateralController.atGoal();
                    }),

            // Final command: Stop the drive when done
            Commands.runOnce(
                () -> {
                  drive.stop();
                  System.out.println("Alignment with AprilTag " + tagId + " complete");
                }));

    alignCommand.addRequirements(drive);
    return alignCommand;
  }

  /**
   * Creates a command that aligns with an AprilTag with additional rotation control
   *
   * @param tagId The ID of the AprilTag to align with
   * @param cameraName The name of the camera to use for vision
   * @param targetLongitudinalDistance The desired forward/backward distance to maintain from the
   *     tag (meters)
   * @param targetLateralDistance The desired side-to-side distance to maintain from the tag
   *     (meters)
   * @param targetYaw The desired yaw angle relative to the AprilTag (degrees)
   * @return A command that aligns the robot with the specified AprilTag
   */
  public Command alignWithAprilTagAndRotation(
      String cameraName,
      double targetLongitudinalDistance,
      double targetLateralDistance,
      double targetYaw) {

    // Create constraints for the motion profiles
    TrapezoidProfile.Constraints longitudinalConstraints = new TrapezoidProfile.Constraints(3.0, 1);
    TrapezoidProfile.Constraints lateralConstraints = new TrapezoidProfile.Constraints(3.0, 1);
    TrapezoidProfile.Constraints rotationConstraints =
        new TrapezoidProfile.Constraints(1.0, 0.8); // rad/s, rad/s²

    // Create ProfiledPIDControllers
    ProfiledPIDController longitudinalController =
        new ProfiledPIDController(2, 0.0, 0, longitudinalConstraints);
    ProfiledPIDController lateralController =
        new ProfiledPIDController(2, 0.0, 0, lateralConstraints);
    ProfiledPIDController rotationController =
        new ProfiledPIDController(3, 0.0, 0, rotationConstraints);

    // Set tolerance for each controller
    longitudinalController.setTolerance(0.05);
    lateralController.setTolerance(0.05);
    rotationController.setTolerance(Math.toRadians(2.0)); // 2 degrees tolerance

    // Enable continuous input for rotation controller (handles angle wrapping)
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    Command alignCommand =
        Commands.sequence(
            // Initialize controllers
            Commands.runOnce(
                () -> {
                  longitudinalController.reset(0);
                  lateralController.reset(0);
                  rotationController.reset(0);
                }),

            // Main alignment command
            Commands.run(
                    () -> {
                      // Get current distances to the tag
                      Optional<Double> longDistOpt = vision.getLongitudinalDistance(cameraName);
                      Optional<Double> latDistOpt = vision.getLateralDistance(cameraName);
                      Optional<Double> tagAngleOpt = vision.getTagHorizontalAngle(cameraName);

                      if (longDistOpt.isEmpty() || latDistOpt.isEmpty() || tagAngleOpt.isEmpty()) {
                        // Tag not visible, stop or rotate to search
                        drive.stop();
                        System.out.println("AprilTag not visible");
                        return;
                      }

                      // Calculate errors
                      double longitudinalError = longDistOpt.get() - targetLongitudinalDistance;
                      double lateralError = latDistOpt.get() - targetLateralDistance;

                      // For rotation, use the horizontal angle to the tag
                      double currentYawRadians = Math.toRadians(tagAngleOpt.get());
                      double targetYawRadians = Math.toRadians(targetYaw);
                      double rotationError = currentYawRadians - targetYawRadians;

                      // Calculate drive speeds using ProfiledPIDControllers
                      double forwardSpeed = -longitudinalController.calculate(longitudinalError, 0);
                      double strafeSpeed = -lateralController.calculate(lateralError, 0);
                      double rotationSpeed = -rotationController.calculate(rotationError, 0);

                      // Apply deadbands
                      if (Math.abs(forwardSpeed) < 0.05 && longitudinalController.atGoal()) {
                        forwardSpeed = 0;
                      }
                      if (Math.abs(strafeSpeed) < 0.05 && lateralController.atGoal()) {
                        strafeSpeed = 0;
                      }
                      if (Math.abs(rotationSpeed) < 0.05 && rotationController.atGoal()) {
                        rotationSpeed = 0;
                      }

                      // Create robot-relative ChassisSpeeds
                      ChassisSpeeds speeds =
                          new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed);
                      drive.runVelocity(speeds);
                    })
                .until(
                    () -> {
                      // Check if we're within tolerance
                      Optional<Double> longDistOpt = vision.getLongitudinalDistance(cameraName);
                      Optional<Double> latDistOpt = vision.getLateralDistance(cameraName);
                      Optional<Double> tagAngleOpt = vision.getTagHorizontalAngle(cameraName);

                      if (longDistOpt.isEmpty() || latDistOpt.isEmpty() || tagAngleOpt.isEmpty()) {
                        return false; // Can't end if we don't see the tag
                      }

                      double longitudinalError = longDistOpt.get() - targetLongitudinalDistance;
                      double lateralError = latDistOpt.get() - targetLateralDistance;

                      // For rotation
                      double currentYawRadians = Math.toRadians(tagAngleOpt.get());
                      double targetYawRadians = Math.toRadians(targetYaw);
                      double rotationError = currentYawRadians - targetYawRadians;

                      // Update controllers to check if at goal
                      longitudinalController.calculate(longitudinalError, 0);
                      lateralController.calculate(lateralError, 0);
                      rotationController.calculate(rotationError, 0);

                      return longitudinalController.atGoal()
                          && lateralController.atGoal()
                          && rotationController.atGoal();
                    }),

            // Final command: Stop the drive when done
            Commands.runOnce(
                () -> {
                  drive.stop();
                }));
    alignCommand.addRequirements(drive);
    return alignCommand;
  }

  /**
   * Creates a command that drives to a pose near an AprilTag and then precisely aligns with it
   *
   * @param tagId The ID of the AprilTag to align with
   * @param cameraName The name of the camera to use for vision
   * @param approachPose The pose to drive to before precise alignment (field-relative)
   * @param targetLongitudinalDistance The desired forward/backward distance to maintain from the
   *     tag (meters)
   * @param targetLateralDistance The desired side-to-side distance to maintain from the tag
   *     (meters)
   * @param targetYaw The desired yaw angle relative to the AprilTag (degrees)
   * @return A command that positions the robot near the tag and then precisely aligns with it
   */
  public Command driveToAndAlignWithAprilTag(
      String cameraName,
      Pose2d approachPose,
      double targetLongitudinalDistance,
      double targetLateralDistance,
      double targetYaw) {

    // Create the IDrive command
    IDrive driveCommand = new IDrive(drive, () -> new Translation2d(), () -> 0.0);

    return Commands.sequence(
        // First, drive to the approach pose using IDrive
        driveCommand
            .beforeStarting(() -> drive.setTargetPose(approachPose))
            .until(
                () -> {
                  // Check if we're close enough to the approach pose using IDrive's withinTolerance
                  return driveCommand.withinTolerance(0.1, Rotation2d.fromDegrees(5.0));
                })
            .withTimeout(5.0), // Timeout to prevent getting stuck

        // Then use the precise alignment command
        alignWithAprilTagAndRotation(
            cameraName, targetLongitudinalDistance, targetLateralDistance, targetYaw));
  }

  /**
   * Creates a command that drives to a branch target position relative to an AprilTag
   *
   * @param cameraName The name of the camera to use for vision
   * @param useLeftBranch Whether to use the left branch target (true) or right branch target
   *     (false)
   * @return A command that drives to the branch target position
   */
  public Command driveToBranchTarget(
      String cameraName, boolean useLeftBranch, BooleanSupplier useLevel4) {
    IDrive iDrive = new IDrive(drive);
    AtomicBoolean targetPoseExist = new AtomicBoolean(false);
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  Integer tagID = vision.getTagId(cameraName).orElse(5449);

                  if (tagID == 5449) {
                    System.out.println("Invalid or no tag detected");
                    drive.setTargetPose(null);
                    targetPoseExist.set(false);
                    return;
                  }

                  Pose2d tagPose =
                      DriverStation.getAlliance().get() == Alliance.Red
                          ? FieldConstants.redCenterFaces.get(tagID)
                          : FieldConstants.blueCenterFaces.get(tagID);
                  // Optional<Pose2d>
                  // tagPose=FieldConstants.getReefTagPose(drive.getPose().getTranslation());

                  if (tagPose == null) {
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
                          .transformBy(transformer)
                          .transformBy(new Transform2d(0, 0, Rotation2d.k180deg));
                  drive.setTargetPose(branchPose);
                })
            /*,Commands.either(iDrive, Commands.none(), () -> targetPoseExist.get())*/ )
        .until(() -> !targetPoseExist.get() || iDrive.atGoal());
  }
}
