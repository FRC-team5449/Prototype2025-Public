// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.commands;

import com.team5449.frc2025.RobotState;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.lib.LoggedTunableNumber;
import com.team5449.lib.util.GeomUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("AutoAlign/drivekP", 3.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("AutoAlign/drivekD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("AutoAlign/thetakP", 7.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("AutoAlign/thetakD", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("AutoAlign/controllerLinearTolerance", 0.08);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("AutoAlign/controllerThetaTolerance", Units.degreesToRadians(2.0));
  private static final LoggedTunableNumber toleranceTime =
      new LoggedTunableNumber("AutoAlign/controllerToleranceSecs", 0.5);
  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber("AutoAlign/maxLinearVelocity", Drive.getMaxLinearSpeedMetersPerSec());
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber("AutoAlign/maxLinearAcceleration", Units.feetToMeters(75.0) * 0.4);
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularVelocity", Drive.getMaxAngularSpeedRadPerSec() * 0.8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber("AutoAlign/maxAngularAcceleration", 12 * 0.8);
  private static final LoggedTunableNumber slowLinearVelocity =
      new LoggedTunableNumber("AutoAlign/slowLinearVelocity", 2.25);
  private static final LoggedTunableNumber slowLinearAcceleration =
      new LoggedTunableNumber("AutoAlign/slowLinearAcceleration", 3.0);
  private static final LoggedTunableNumber slowAngularVelocity =
      new LoggedTunableNumber("AutoAlign/slowAngularVelocity", Math.PI / 2.0);
  private static final LoggedTunableNumber slowAngularAcceleration =
      new LoggedTunableNumber("AutoAlign/slowAngularAcceleration", Math.PI);
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoAlign/ffMinRadius", 0.2);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoAlign/ffMaxRadius", 0.8);

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Translation2d> feedforwardSupplier;
  private final BooleanSupplier slowMode;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;
  private final Timer toleranceTimer = new Timer();

  private final Drive drive;

  public AutoAlignCommand(
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      BooleanSupplier slowMode,
      Drive drive) {
    this.poseSupplier = poseSupplier;
    this.feedforwardSupplier = feedforwardSupplier;
    this.slowMode = slowMode;
    this.drive = drive;
    // Set up both controllers
    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());
    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());
  }

  @Override
  public void initialize() {
    toleranceTimer.restart();
    updateConstraints();
    resetControllers();
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        this::updateConstraints,
        maxLinearVelocity,
        maxLinearAcceleration,
        slowLinearVelocity,
        slowLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration,
        slowAngularVelocity,
        slowAngularAcceleration);

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d targetPose = poseSupplier.get();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScalar =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScalar
            + linearController.calculate(currentDistance, 0.0);
    if (linearController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScalar
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;
    if (!linearController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    // Log data
    Logger.recordOutput("AutoAlign/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoAlign/DistanceSetpoint", linearController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/SetpointPose",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/GoalPose", targetPose);

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation()
            .plus(feedforwardSupplier.get());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }

  private void resetControllers() {
    // Reset measurements and velocities
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d goalPose = poseSupplier.get();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    Rotation2d robotToGoalAngle =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity =
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(robotToGoalAngle.unaryMinus())
                .getX());
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  private void updateConstraints() {
    if (slowMode.getAsBoolean()) {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(slowLinearVelocity.get(), slowLinearAcceleration.get()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowAngularVelocity.get(), slowAngularAcceleration.get()));
    } else {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
    }
  }
}
