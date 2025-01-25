// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5449.frc2025.commands.DriveCommands;
import com.team5449.frc2025.subsystems.TunerConstants;
import com.team5449.frc2025.subsystems.arm.Arm;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.drive.GyroIO;
import com.team5449.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team5449.frc2025.subsystems.drive.ModuleIO;
import com.team5449.frc2025.subsystems.drive.ModuleIOSim;
import com.team5449.frc2025.subsystems.drive.ModuleIOTalonFX;
import com.team5449.frc2025.subsystems.elevator.Elevator;
import com.team5449.frc2025.subsystems.endeffector.EndEffector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Arm arm;

  @SuppressWarnings("unused")
  private final RobotState robotState = RobotState.getInstance();

  // private final AprilTagVision aprilTagVision;

  private final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  // private final CommandPS5Controller operatorGamepad = new CommandPS5Controller(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        elevator = new Elevator();

        endEffector = new EndEffector();

        arm = new Arm();
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        elevator = null;
        endEffector = null;
        arm = null;
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = null;
        endEffector = null;
        arm = null;
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("None", Commands.none());
    // autoChooser.addOption(
    //     "Elevator Characterization",
    //     new StaticCharacterizationCommand(
    //             elevator, (current) -> elevator.runCharacterization(current),
    // elevator::getVelocity)
    //         .finallyDo(elevator::endCharacterization));
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverGamepad.getLeftY(),
            () -> -driverGamepad.getLeftX(),
            () -> -driverGamepad.getRightX()));

    driverGamepad
        .circle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverGamepad.getLeftY(),
                () -> -driverGamepad.getLeftX(),
                () -> new Rotation2d()));

    // Reset gyro to 0 when triangle is pressed
    driverGamepad
        .triangle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverGamepad
        .pov(0)
        .and(arm::notStowed)
        .onTrue(elevator.positionCommand(Elevator.Goal.LEVEL_1.targetRotation.get()));

    driverGamepad
        .pov(90)
        .and(arm::notStowed)
        .onTrue(elevator.positionCommand(Elevator.Goal.LEVEL_2.targetRotation.get()));

    driverGamepad
        .pov(180)
        .and(arm::notStowed)
        .onTrue(elevator.positionCommand(Elevator.Goal.LEVEL_3.targetRotation.get()));

    driverGamepad
        .pov(270)
        .and(arm::notStowed)
        .onTrue(elevator.positionCommand(Elevator.Goal.LEVEL_4.targetRotation.get()));

    driverGamepad
        .R1()
        .onTrue(
            elevator
                .positionCommand(Elevator.Goal.IDLE.targetRotation.get())
                .andThen(
                    Commands.waitUntil(elevator::isStowed)
                        .andThen(arm.positionCommand(Arm.Goal.STOW))))
        .and(arm::isStowed)
        .whileTrue(
            Commands.runEnd(
                () -> endEffector.runOpenLoop(-0.5), () -> endEffector.runOpenLoop(0), endEffector))
        .onFalse(arm.positionCommand(Arm.Goal.IDLE));

    driverGamepad
        .L1()
        .and(elevator::atGoal)
        .whileTrue(
            Commands.runEnd(
                () -> endEffector.runOpenLoop(-0.5),
                () -> endEffector.runOpenLoop(0),
                endEffector));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
