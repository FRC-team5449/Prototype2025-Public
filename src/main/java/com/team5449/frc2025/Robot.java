// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.team5449.lib.LoggedTunableNumber;
import com.team5449.lib.thirdpartylibs.Elastic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private final LoggedTunableNumber poseX = new LoggedTunableNumber("RobotState/Pose_X", 0.0);
  private final LoggedTunableNumber poseY = new LoggedTunableNumber("RobotState/Pose_Y", 0.0);
  private final LoggedTunableNumber poseRotation =
      new LoggedTunableNumber("RobotState/Pose_Rotation_Degree", 0.0);
  private final LoggedTunableNumber targetX = new LoggedTunableNumber("targetPose/X", 0);
  private final LoggedTunableNumber targetY = new LoggedTunableNumber("targetPose/Y", 0);
  private final LoggedTunableNumber targetT = new LoggedTunableNumber("targetPose/T", 0);

  public Robot() {
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            RobotState.getInstance()
                .setPose(
                    new Pose2d(
                        poseX.get(), poseY.get(), Rotation2d.fromDegrees(poseRotation.get()))),
        poseX,
        poseY,
        poseRotation);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            RobotState.getInstance()
                .setTargetPose(
                    new Pose2d(
                        targetX.get(), targetY.get(), Rotation2d.fromDegrees(targetT.get()))),
        targetX,
        targetY,
        targetT);
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    Elastic.selectTab("Autonomous");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Elastic.selectTab("Teleoperated");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.periodic();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
