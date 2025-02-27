// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.auto;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorSubsystem;
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

  public Command dummyFourLV3() {
    var startToReef1 = getAutoPath("upperStartToReefI");
    var reef1ToSource = getAutoPath("reefIToSource");
    var sourceToReef2 = getAutoPath("sourceToReefJ");
    var reef2ToSource = getAutoPath("reefJToSource");
    var sourceToReef3 = getAutoPath("sourceToReefK");
    return Commands.sequence(
        startAt(startToReef1),
        AutoBuilder.followPath(startToReef1)
            .alongWith(Commands.waitSeconds(0.8).andThen(elevator.setStateOk(ElevatorState.L3))),
        endEffector.outtake().withTimeout(0.5),
        elevator.setState(ElevatorState.IDLE),
        AutoBuilder.followPath(reef1ToSource),
        waitSeconds(1),
        AutoBuilder.followPath(sourceToReef2)
            .alongWith(Commands.waitSeconds(1.5).andThen(elevator.setStateOk(ElevatorState.L3))),
        endEffector.outtake().withTimeout(0.5),
        elevator.setState(ElevatorState.IDLE),
        AutoBuilder.followPath(reef2ToSource),
        waitSeconds(1),
        AutoBuilder.followPath(sourceToReef3)
            .alongWith(Commands.waitSeconds(1.1).andThen(elevator.setStateOk(ElevatorState.L3))),
        endEffector.outtake().withTimeout(0.5),
        elevator.setStateOk(ElevatorState.IDLE));
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

  private Command startAt(PathPlannerPath firstPath) {
    return Commands.runOnce(() -> drive.setPose(firstPath.getStartingHolonomicPose().get()));
  }
}
