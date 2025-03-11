// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem.ArmState;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorSubsystem;
import com.team5449.lib.util.AllianceFlipUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoFactory {
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final EndEffectorSubsystem endEffector;
  private final AutoCommand autoCommand;

  public Command dummyFourLV3() {
    var startToReef1 = getAutoPath("upperStartToReefI");
    var reef1ToSource = getAutoPath("reefIToSource");
    var sourceToReef2 = getAutoPath("sourceToReefK");
    var reef2ToSource = getAutoPath("reefKToSource");
    var sourceToReef3 = getAutoPath("sourceToReefL");
    return Commands.sequence(
        startAt(startToReef1),
        AutoBuilder.followPath(startToReef1),
        autoCommand.driveToBranchTarget("limelight", false, () -> true),
        extendArmAndElevate(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        Commands.parallel(
            AutoBuilder.followPath(reef1ToSource)
                .finallyDo(
                    () -> {
                      System.out.println("Path Finishshshshshshshshshsh");
                      drive.stop();
                    }),
            stowElevatorAndArm().andThen(arm.setState(ArmState.INTAKE))),
        endEffector.intake(),
        AutoBuilder.followPath(sourceToReef2),
        autoCommand.driveToBranchTarget("limelight", true, () -> true),
        extendArmAndElevate(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        Commands.sequence(stowElevatorAndArm().andThen(arm.setState(ArmState.INTAKE))),
        Commands.parallel(
            AutoBuilder.followPath(reef2ToSource)
                .finallyDo(
                    () -> {
                      System.out.println("Path Finishshshshshshshshshsh");
                      drive.stop();
                    }),
            stowElevatorAndArm().andThen(arm.setState(ArmState.INTAKE))),
        endEffector.intake(),
        AutoBuilder.followPath(sourceToReef3),
        autoCommand.driveToBranchTarget("limelight", false, () -> true),
        extendArmAndElevate(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        stowElevatorAndArm());
  }

  public Command poor() {
    var midStartToReefH = getAutoPath("middleStartToReefH");
    return Commands.sequence(
        startAt(midStartToReefH),
        AutoBuilder.followPath(midStartToReefH).alongWith(elevator.setStateOk(ElevatorState.L3)),
        endEffector.outtake().withTimeout(0.5),
        elevator.setStateOk(ElevatorState.IDLE));
  }

  public Command autoPathTry() {
    return startAt(getAutoPath("left")).andThen(AutoBuilder.followPath(getAutoPath("left")));
  }

  private PathPlannerPath getAutoPath(String fileName) {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
    return path;
  }

  public Command extendArmAndElevate(ElevatorState state) {
    return arm.setStateOk(ArmState.IDLE).andThen(elevator.setStateOk(state));
  }

  public Command stowElevatorAndArm() {
    return arm.setStateOk(ArmState.IDLE).andThen(elevator.setStateOk(ElevatorState.IDLE));
  }

  private Command startAt(PathPlannerPath firstPath) {
    return Commands.runOnce(
        () -> drive.setPose(AllianceFlipUtil.apply(firstPath.getStartingHolonomicPose().get())));
  }
}
