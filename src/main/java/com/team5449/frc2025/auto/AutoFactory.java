// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
    PathPlannerPath startToReef1 = getAutoPath("startToReef1");
    PathPlannerPath reef1ToSource = getAutoPath("reef1ToSource");
    PathPlannerPath sourceToReef2 = getAutoPath("sourceToReef2");
    PathPlannerPath reef2ToSource = getAutoPath("reef2ToSource");
    PathPlannerPath sourceToReef3 = getAutoPath("sourceToReef3");
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startToReef1.getStartingHolonomicPose().get())),
        AutoBuilder.followPath(startToReef1),
        elevator.setStateCommand(ElevatorState.LEVEL_3),
        Commands.waitSeconds(0.5),
        endEffector.outtake().withTimeout(1.5),
        Commands.waitSeconds(0.5),
        elevator.setStateCommand(ElevatorState.IDLE) // ,
        // AutoBuilder.followPath(reef1ToSource),

        );
  }

  public Command autoPathTry() {
    return new PathPlannerAuto("autoTry");
  }

  public PathPlannerPath getAutoPath(String fileName) {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
    return path;
  }
}
