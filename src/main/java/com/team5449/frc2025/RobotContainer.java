// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  // private final CommandPS5Controller operatorGamepad = new CommandPS5Controller(0);

  // private final LoggedDashboardChooser<Command> autoChooser;

  // private final AutoCommand autoCommand;

  public RobotContainer() {}

  // private void configureBindings() {
  //   drive.setDefaultCommand(
  //       DriveCommands.joystickDrive(
  //           drive,
  //           () -> driverGamepad.getLeftY(),
  //           () -> driverGamepad.getLeftX(),
  //           () -> -driverGamepad.getRightX()));

  //   driverGamepad
  //       .circle()
  //       .whileTrue(
  //           DriveCommands.joystickDriveAtAngle(
  //               drive,
  //               () -> -driverGamepad.getLeftY(),
  //               () -> -driverGamepad.getLeftX(),
  //               Rotation2d::new));

  //   // Reset gyro to 0 when triangle is pressed
  //   driverGamepad.square().whileTrue(new IDrive(drive));

  //   driverGamepad
  //       .triangle()
  //       .onTrue(
  //           Commands.runOnce(
  //                   () ->
  //                       drive.setPose(
  //                           new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
  //                   drive)
  //               .ignoringDisable(true));

  //   driverGamepad
  //       .cross()
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> drive.setPose(new Pose2d(new Translation2d(), drive.getRotation())),
  // drive));

  //   driverGamepad.pov(0).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L4));

  //   driverGamepad.pov(90).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L3));

  //   driverGamepad.pov(180).and(() ->
  // !arm.intaking()).onTrue(elevator.setState(ElevatorState.L1));

  //   driverGamepad.pov(270).and(() ->
  // !arm.intaking()).onTrue(elevator.setState(ElevatorState.L2));

  //   driverGamepad
  //       .R1()
  //       .onTrue(
  //           elevator
  //               .setState(ElevatorState.IDLE)
  //               .alongWith(
  //                   Commands.waitUntil(elevator::isStowed)
  //
  // .andThen(arm.setStateCommand(ArmState.INTAKE).onlyIf(driverGamepad.R1()))))
  //       .onFalse(arm.setStateCommand(ArmState.IDLE))
  //       .and(arm::intaking)
  //       .whileTrue(endEffector.intake());

  //   driverGamepad
  //       .R2()
  //       .and(
  //           () ->
  //               elevator.atGoal(ElevatorState.L4)
  //                   || elevator.atGoal(ElevatorState.L3)
  //                   || elevator.atGoal(ElevatorState.L1))
  //       .whileTrue(
  //           new StartEndCommand(
  //               () -> arm.setDesiredState(ArmState.SCORE),
  //               () -> arm.setDesiredState(ArmState.IDLE)));

  //   driverGamepad.L1().and(elevator::atGoal).whileTrue(endEffector.outtake());

  //   // driverGamepad.L2().whileTrue(endEffector.reverse());
  //   driverGamepad.L2().whileTrue(autoCommand.alignWithAprilTagAndRotation("limelight", 0.1, 0,
  // 0));

  //   // operatorGamepad.pov(0).onTrue(climber.setState(ClimberState.IDLE));
  //   // operatorGamepad.pov(90).onTrue(climber.setState(ClimberState.ALIGN));
  //   // operatorGamepad.pov(180).onTrue(climber.setState(ClimberState.CLIMB));

  //   // operatorGamepad.L1().whileTrue(climber.elevate());
  //   // operatorGamepad.R1().whileTrue(climber.decline());
  // }

  // public void periodic() {
  //   if (elevator.getDesiredState() == ElevatorState.L4) {
  //     drive.setSlowMode(true);
  //   } else {
  //     drive.setSlowMode(false);
  //   }
  // }

  public Command getAutonomousCommand() {
    return Commands.print("Naaaaa");
  }
}
