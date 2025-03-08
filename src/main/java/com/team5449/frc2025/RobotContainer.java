// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team5449.frc2025.auto.AutoCommand;
import com.team5449.frc2025.auto.AutoFactory;
import com.team5449.frc2025.commands.DriveCommands;
import com.team5449.frc2025.commands.IDrive;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision.CameraConfig;
import com.team5449.frc2025.subsystems.arm.ArmSimTalonIO;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem.ArmState;
import com.team5449.frc2025.subsystems.arm.ArmTalonIO;
import com.team5449.frc2025.subsystems.climber.ClimberConstants;
import com.team5449.frc2025.subsystems.climber.ClimberSubsystem;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.drive.GyroIO;
import com.team5449.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team5449.frc2025.subsystems.drive.ModuleIO;
import com.team5449.frc2025.subsystems.drive.ModuleIOSim;
import com.team5449.frc2025.subsystems.drive.ModuleIOTalonFX;
import com.team5449.frc2025.subsystems.drive.TunerConstants;
import com.team5449.frc2025.subsystems.elevator.ElevatorConstants;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorIONEO;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorIOSim;
import com.team5449.frc2025.subsystems.endeffector.EndEffectorSubsystem;
import com.team5449.frc2025.subsystems.hopper.HopperConstants;
import com.team5449.frc2025.subsystems.hopper.HopperSubsystem;
import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.SimTalonFXIO;
import com.team5449.lib.subsystems.TalonFXIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final EndEffectorSubsystem endEffector;
  private final ArmSubsystem arm;
  private final HopperSubsystem hopper;

  @SuppressWarnings("unused")
  private final ClimberSubsystem climber;

  @SuppressWarnings("unused")
  private final AprilTagVision vision;

  //   private final CameraConfig cameraConfig1;

  //   private final CameraConfig[] cameraConfigs;

  @SuppressWarnings("unused")
  private final RobotState robotState = RobotState.getInstance();

  private final AutoFactory autoFactory;

  private final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);

  @SuppressWarnings("unused")
  private final CommandPS5Controller operatorGamepad = new CommandPS5Controller(1);

  // private final CommandPS5Controller operatorGamepad = new CommandPS5Controller(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  private final AutoCommand autoCommand;

  public RobotContainer() {
    // Check constructor for heavy initialization

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

        climber = new ClimberSubsystem(new TalonFXIO(ClimberConstants.kClimberConfig));

        vision =
            new AprilTagVision(
                new CameraConfig[] {
                  new CameraConfig(
                      "limelight", new Transform3d(0.35, 0, 0.175, new Rotation3d(0, 15, 0)), 0.08)
                });
        // vision = null;

        hopper = new HopperSubsystem(new TalonFXIO(HopperConstants.kHopperConfig));
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

        climber = new ClimberSubsystem(new SimTalonFXIO(ClimberConstants.kClimberConfig));

        // cameraConfig1 = null;
        // cameraConfigs = null;
        vision = null;

        hopper = new HopperSubsystem(new SimTalonFXIO(HopperConstants.kHopperConfig));
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

        climber = new ClimberSubsystem(new MotorIO() {});

        // cameraConfig1 = null;
        // cameraConfigs = null;
        vision = null;

        hopper = new HopperSubsystem(new MotorIO() {});
        break;
    }

    autoFactory = new AutoFactory(drive, elevator, arm, endEffector);
    autoCommand = new AutoCommand(drive, elevator, arm, endEffector, vision);

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

    // Reset gyro to 0 when triangle is pressed
    driverGamepad.square().whileTrue(new IDrive(drive));

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

    driverGamepad.pov(0).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L4));

    driverGamepad.pov(90).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L3));

    driverGamepad.pov(180).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L1));

    driverGamepad.pov(270).and(() -> !arm.intaking()).onTrue(elevator.setState(ElevatorState.L2));

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
        .whileTrue(endEffector.intake());

    driverGamepad
        .R2()
        .and(
            () ->
                elevator.atGoal(ElevatorState.L4)
                    || elevator.atGoal(ElevatorState.L3)
                    || elevator.atGoal(ElevatorState.L1))
        .whileTrue(
            new StartEndCommand(
                () -> arm.setDesiredState(ArmState.SCORE),
                () -> arm.setDesiredState(ArmState.IDLE)));

    driverGamepad
        .L1()
        .and(() -> !elevator.atGoal(ElevatorState.L1) && elevator.atGoal() && !elevator.isStowed())
        .whileTrue(endEffector.outtake());

    driverGamepad
        .L1()
        .and(() -> elevator.atGoal(ElevatorState.L1))
        .whileTrue(endEffector.l1Outtake());

    // driverGamepad.L2().whileTrue(endEffector.reverse());
    driverGamepad.L2().onTrue(autoCommand.driveToBranchTarget("limelight", false));
    // driverGamepad.L2().whileTrue(autoCommand.alignWithAprilTagAndRotation("limelight", 0.1, 0,
    // 0));

    // operatorGamepad.pov(0).onTrue(climber.setState(ClimberState.IDLE));
    // operatorGamepad.pov(90).onTrue(climber.setState(ClimberState.ALIGN));
    // operatorGamepad.pov(180).onTrue(climber.setState(ClimberState.CLIMB));

    operatorGamepad.L1().whileTrue(climber.elevate());
    operatorGamepad.R1().whileTrue(climber.decline());

    operatorGamepad.L2().whileTrue(hopper.elevate());
    operatorGamepad.R2().whileTrue(hopper.decline());
  }

  public void periodic() {
    if (elevator.getDesiredState() == ElevatorState.L4) {
      drive.setSlowMode(true);
    } else {
      drive.setSlowMode(false);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
