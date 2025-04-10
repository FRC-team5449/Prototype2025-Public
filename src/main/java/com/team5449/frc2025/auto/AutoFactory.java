// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.auto;

import static com.pathplanner.lib.auto.AutoBuilder.followPath;
import static edu.wpi.first.wpilibj2.command.Commands.run;

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
  private final AlignCommands autoCommand;

  public Command dummyFourLV3() {
    var startToReef1 = getAutoPath("upperStartToReefI");
    var reef1ToSource = getAutoPath("reefIToSource");
    var sourceToReef2 = getAutoPath("sourceToReefK");
    var reef2ToSource = getAutoPath("reefKToSource");
    var sourceToReef3 = getAutoPath("sourceToReefL");
    return Commands.sequence(
        startAt(startToReef1),
        followPath(startToReef1),
        autoCommand.driveToBranchTarget(false, () -> true),
        extendElevator(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        stowElevator().andThen(arm.setState(ArmState.INTAKE)),
        followPath(reef1ToSource),
        endEffector.intake().alongWith(Commands.runOnce(drive::stop)),
        followPath(sourceToReef2),
        autoCommand.driveToBranchTarget(true, () -> true),
        extendElevator(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        stowElevator().andThen(arm.setState(ArmState.INTAKE)),
        followPath(reef2ToSource),
        endEffector.intake().alongWith(Commands.runOnce(drive::stop)),
        followPath(sourceToReef3),
        autoCommand.driveToBranchTarget(false, () -> true),
        extendElevator(ElevatorState.L4),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        stowElevator());
  }

  // spotless:off
  public Command fastAss3Level4Upper() {
    var startToReef1 = getAutoPath("upperStartToReefI");
    var reef1ToSource = getAutoPath("reefIToSource");
    var sourceToReef2 = getAutoPath("sourceToReefK");
    var reef2ToSource = getAutoPath("reefKToSource");
    var sourceToReef3 = getAutoPath("sourceToReefL");

    return Commands.sequence(
        startAt(startToReef1),
        followPath(startToReef1).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(false, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        score(),
        Commands.parallel(
            followPathStop(reef1ToSource),
            stowElevator().andThen(arm.setState(ArmState.INTAKE))),
        run(drive::stop).withDeadline(endEffector.intake()),
        followPath(sourceToReef2).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(true, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        score(),
        Commands.parallel(
            followPathStop(reef2ToSource),
            stowElevator().andThen(arm.setState(ArmState.INTAKE))),
        run(drive::stop).withDeadline(endEffector.intake()),
        followPath(sourceToReef3).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(false, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        score(),
        stowElevator());
  }
  // spotless:on

  // spotless:off
  public Command fastAss3Level4Lower() {
    var startToReef1 = getAutoPath("lowerStartToReefE");
    var reef1ToSource = getAutoPath("reefEToSource");
    var sourceToReef2 = getAutoPath("sourceToReefD");
    var reef2ToSource = getAutoPath("reefDToSource");
    var sourceToReef3 = getAutoPath("sourceToReefC");

    return Commands.sequence(
        startAt(startToReef1),
        followPath(startToReef1).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(true, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        Commands.parallel(
            followPathStop(reef1ToSource),
            stowElevator().andThen(arm.setState(ArmState.INTAKE))),
        run(drive::stop).withDeadline(endEffector.intake()),
        followPath(sourceToReef2).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(false, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        Commands.parallel(
            followPathStop(reef2ToSource),
            stowElevator().andThen(arm.setState(ArmState.INTAKE))),
        run(drive::stop).withDeadline(endEffector.intake()),
        followPath(sourceToReef3).alongWith(arm.setState(ArmState.IDLE)),
        autoCommand
            .driveToBranchTarget(true, () -> true)
            .alongWith(extendElevator(ElevatorState.L4)),
        arm.setStateOk(ArmState.SCORE),
        endEffector.outtakeAuto(),
        stowElevator());
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

  private Command followPathStop(PathPlannerPath path) {
    return followPath(path).andThen(drive::stop);
  }

  private Command extendElevator(ElevatorState state) {
    return arm.setStateOk(ArmState.IDLE).andThen(elevator.setStateOk(state));
  }

  private Command stowElevator() {
    return arm.setStateOk(ArmState.IDLE).andThen(elevator.setStateOk(ElevatorState.IDLE));
  }
  
  private Command score(){
    return arm.setStateOk(ArmState.SCORE).alongWith(Commands.waitSeconds(0.3).andThen(endEffector.outtakeAuto()));
  }

  private Command startAt(PathPlannerPath firstPath) {
    return Commands.runOnce(
        () -> drive.setPose(AllianceFlipUtil.apply(firstPath.getStartingHolonomicPose().get())));
  }
}
