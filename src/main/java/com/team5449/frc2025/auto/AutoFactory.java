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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoFactory {
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;

  public Command dummyFourLV3() {
    PathPlannerPath startToA = getAutoPath("Example Auto");
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startToA.getStartingHolonomicPose().get())),
        AutoBuilder.followPath(startToA));
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
