// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.team5449.lib.thirdpartylibs.LimelightHelpers;
import com.team5449.lib.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
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

  private final CANBus canBus;
  private final Timer threadTimer;
  private final RobotContainer robotContainer;

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
    threadTimer = new Timer();
    threadTimer.start();
    FieldConstants.initializeField();
    robotContainer = new RobotContainer();

    //TODO Ensure the mode you are using is correct
    LimelightHelpers.SetIMUMode("limelight", 1);
    LimelightHelpers.SetRobotOrientation("limelight", FieldConstants.initialHeading, 0, 0, 0, 0, 0);

    RobotState.getInstance()
        .setPose(
            DriverStation.getAlliance().get() == Alliance.Red
                ? new Pose2d()
                : new Pose2d(0, 0, Rotation2d.k180deg));

    canBus = new CANBus("*");
    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    if (threadTimer.hasElapsed(20)) {
      Threads.setCurrentThreadPriority(true, 10);
    }

    robotContainer.periodic();
    CommandScheduler.getInstance().run();
    Logger.recordOutput("canivore/BusOffCount", canBus.getStatus().BusOffCount);
    Logger.recordOutput("canivore/BusUtilization", canBus.getStatus().BusUtilization);
    Logger.recordOutput("canivore/REC", canBus.getStatus().REC);
    Logger.recordOutput("canivore/TEC", canBus.getStatus().TEC);
    Logger.recordOutput("canivore/TxFullCount", canBus.getStatus().TxFullCount);
  }

  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    robotContainer.disblePeriodic();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    // LimelightHelpers.SetIMUMode("limelight", 2);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotContainer.setFlip(AllianceFlipUtil.shouldFlip() ? -1 : 1);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // LimelightHelpers.SetIMUMode("limelight", 2);
  }

  @Override
  public void teleopPeriodic() {}

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
