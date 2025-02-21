// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team5449.frc2025.auto.AutoFactory;
import com.team5449.frc2025.commands.AutoAlignCommand;
import com.team5449.frc2025.commands.DriveCommands;
import com.team5449.frc2025.subsystems.TunerConstants;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision.CameraConfig;
import com.team5449.frc2025.subsystems.arm.ArmSimTalonIO;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem.ArmState;
import com.team5449.frc2025.subsystems.arm.ArmTalonIO;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.drive.GyroIO;
import com.team5449.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team5449.frc2025.subsystems.drive.ModuleIO;
import com.team5449.frc2025.subsystems.drive.ModuleIOSim;
import com.team5449.frc2025.subsystems.drive.ModuleIOTalonFX;
import com.team5449.frc2025.subsystems.elevator.ElevatorConstants;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorIONEO;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorIOSim;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorSubsystem;
import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.SimTalonFXIO;
import com.team5449.lib.subsystems.TalonFXIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final EndEffectorSubsystem endEffector;
  private final ArmSubsystem arm;

  @SuppressWarnings("unused")
  private final AprilTagVision vision;

  private final CameraConfig cameraConfig1;

  private final CameraConfig[] cameraConfigs;

  @SuppressWarnings("unused")
  private final RobotState robotState = RobotState.getInstance();

  private final AutoFactory autoFactory;

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

        elevator = new ElevatorSubsystem(new TalonFXIO(ElevatorConstants.kElevatorConfig));

        endEffector = new EndEffectorSubsystem(new EndEffectorIONEO());

        arm = new ArmSubsystem(new ArmTalonIO());

        cameraConfig1 =
            new CameraConfig(
                "limelight-front",
                new Transform3d(new Translation3d(), new Rotation3d()),
                // new Rotation3d(Degree.of(90), Degree.of(0), Degree.of(180))),
                5);
        // TODO finish this
        cameraConfigs = new CameraConfig[] {cameraConfig1};
        vision = new AprilTagVision(cameraConfigs);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        elevator = new ElevatorSubsystem(new SimTalonFXIO(ElevatorConstants.kElevatorConfig));

        endEffector = new EndEffectorSubsystem(new EndEffectorIOSim());

        arm = new ArmSubsystem(new ArmSimTalonIO());

        cameraConfig1 = null;
        cameraConfigs = null;
        vision = null;
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = new ElevatorSubsystem(new MotorIO() {});

        endEffector = null;

        arm = new ArmSubsystem(new MotorIO() {});

        cameraConfig1 = null;
        cameraConfigs = null;
        vision = null;
        break;
    }

    autoFactory = new AutoFactory(drive, elevator, arm, endEffector);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>());
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Dummy 4 Level3", autoFactory.dummyFourLV3());
    autoChooser.addOption("Auto Try", autoFactory.autoPathTry());
    autoChooser.addOption("test", new PathPlannerAuto("New Auto"));
    autoChooser.addOption("mid", autoFactory.poor());
    // File autoDir = new File("/deploy/pathplanner/autos");
    // for (File file : autoDir.listFiles()) {
    //   autoChooser.addOption(file.getName(), new PathPlannerAuto(file.getName()));
    // }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverGamepad.getLeftY(),
            () -> driverGamepad.getLeftX(),
            () -> -driverGamepad.getRightX()));

    driverGamepad
        .circle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverGamepad.getLeftY(),
                () -> -driverGamepad.getLeftX(),
                Rotation2d::new));

    // Reset gyro to 0 when triangle is pressed
    driverGamepad
        .square()
        .whileTrue(new AutoAlignCommand(() -> new Translation2d(), () -> false, drive));
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
        .cross()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(new Translation2d(), drive.getRotation())), drive));
    driverGamepad.pov(0).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L1));

    driverGamepad.pov(90).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L2));

    driverGamepad.pov(180).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L3));

    driverGamepad.pov(270).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L4));

    driverGamepad
        .R1()
        .onTrue(
            elevator
                .setState(ElevatorState.IDLE)
                .alongWith(
                    Commands.waitUntil(elevator::isStowed)
                        .andThen(arm.setStateCommand(ArmState.INTAKE).onlyIf(driverGamepad.R1()))))
        .onFalse(arm.setStateCommand(ArmState.IDLE))
        .and(arm::intaking)
        .whileTrue(endEffector.outtake());

    driverGamepad.L1().and(elevator::atGoal).whileTrue(endEffector.outtake());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
