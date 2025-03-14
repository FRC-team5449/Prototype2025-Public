// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025;

import com.team5449.frc2025.auto.AutoCommand;
import com.team5449.frc2025.auto.AutoFactory;
import com.team5449.frc2025.commands.DriveCommands;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision;
import com.team5449.frc2025.subsystems.apriltagvision.AprilTagVision.CameraConfig;
import com.team5449.frc2025.subsystems.arm.ArmSimTalonIO;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.arm.ArmSubsystem.ArmState;
import com.team5449.frc2025.subsystems.arm.ArmTalonIO;
import com.team5449.frc2025.subsystems.climber.ClimberConstants;
import com.team5449.frc2025.subsystems.climber.ClimberSubsystem;
import com.team5449.frc2025.subsystems.climber.ClimberSubsystem.ClimberState;
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
import com.team5449.frc2025.subsystems.hopper.HopperSubsystem.HopperState;
import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.SimTalonFXIO;
import com.team5449.lib.subsystems.TalonFXIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lombok.Setter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  @Setter private double flip = 1;
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final EndEffectorSubsystem endEffector;
  private final ArmSubsystem arm;
  private final HopperSubsystem hopper;

  private final ClimberSubsystem climber;

  private final AprilTagVision vision;

  private final RobotState robotState = RobotState.getInstance();

  private final AutoFactory autoFactory;

  private final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);

  private boolean useLevel4 = true;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final AutoCommand autoCommand;

  private DriveMode currentMode = DriveMode.TELEOP;

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

    autoCommand = new AutoCommand(drive, elevator, arm, endEffector, vision);

    autoFactory = new AutoFactory(drive, elevator, arm, endEffector, autoCommand);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>());
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Dummy 4 Level3", autoFactory.dummyFourLV3());
    autoChooser.addOption("Fast Ass 3 Level4 Upper", autoFactory.fastAss3Level4Upper());
    autoChooser.addOption("Fast Ass 3 Level4 Lower", autoFactory.fastAss3Level4Lower());
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverGamepad.getLeftY() * flip,
            () -> driverGamepad.getLeftX() * flip,
            () -> -driverGamepad.getRightX()));

    driverGamepad
        .circle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverGamepad.getLeftY() * flip,
                () -> -driverGamepad.getLeftX() * flip,
                Rotation2d::new));

    driverGamepad
        .pov(0)
        .and(() -> currentMode == DriveMode.TELEOP)
        .onTrue(setElevatorState(ElevatorState.L4));

    driverGamepad
        .pov(90)
        .and(() -> currentMode == DriveMode.TELEOP)
        .onTrue(setElevatorState(ElevatorState.L3));

    driverGamepad
        .pov(180)
        .and(() -> currentMode == DriveMode.TELEOP)
        .onTrue(setElevatorState(ElevatorState.L1));

    driverGamepad
        .pov(270)
        .and(() -> currentMode == DriveMode.TELEOP)
        .onTrue(setElevatorState(ElevatorState.L2));

    driverGamepad
        .R1()
        .onTrue(
            elevator
                .setState(ElevatorState.IDLE)
                .andThen(Commands.print("Done state setting"))
                .alongWith(
                    Commands.waitUntil(elevator::isStowed).andThen(arm.setState(ArmState.INTAKE))));

    driverGamepad.R1().and(() -> arm.intaking() && currentMode == DriveMode.TELEOP).whileTrue(endEffector.intake());

    // driverGamepad.touchpad().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

    driverGamepad
        .R2()
        .and(() -> !elevator.atGoal(ElevatorState.IDLE) && currentMode == DriveMode.TELEOP)
        .whileTrue(
            new StartEndCommand(
                () -> arm.setDesiredState(ArmState.SCORE),
                () -> arm.setDesiredState(ArmState.IDLE)));

    driverGamepad
        .L1()
        .and(() -> !elevator.atGoal(ElevatorState.L1) && elevator.atGoal() && !elevator.isStowed())
        .whileTrue(endEffector.outtake());

    driverGamepad.L1().and(elevator::isStowed).whileTrue(endEffector.outtake());

    driverGamepad
        .L1()
        .and(() -> elevator.atGoal(ElevatorState.L1))
        .whileTrue(endEffector.l1Outtake());

    driverGamepad
        .L2()
        .and(() -> elevator.isStowed() && currentMode == DriveMode.TELEOP)
        .onTrue(
            autoCommand
                .driveToBranchTarget("limelight", true, () -> useLevel4)
                .until(driverGamepad.create()));

    driverGamepad
        .R2()
        .and(() -> elevator.isStowed() && currentMode == DriveMode.TELEOP)
        .onTrue(
            autoCommand
                .driveToBranchTarget("limelight", false, () -> useLevel4)
                .until(driverGamepad.create()));

    driverGamepad.options().onTrue(Commands.runOnce(() -> useLevel4 = !useLevel4));

    driverGamepad
        .triangle()
        .and(() -> elevator.isStowed() && climber.atGoal(ClimberState.IDLE))
        .onTrue(
            climber
                .setStateOk(ClimberState.ALIGN)
                .alongWith(Commands.runOnce(() -> currentMode = DriveMode.CLIMB))
                .andThen(hopper.setState(HopperState.FOLD)));

    driverGamepad
        .triangle()
        .and(
            () ->
                elevator.isStowed()
                    && climber.atGoal(ClimberState.ALIGN)
                    && currentMode == DriveMode.CLIMB)
        .onTrue(climber.setState(ClimberState.IDLE));

    driverGamepad.R2().and(() -> currentMode == DriveMode.CLIMB).whileTrue(climber.elevate());

    driverGamepad.L2().and(() -> currentMode == DriveMode.CLIMB).whileTrue(climber.decline());

    driverGamepad
        .cross()
        .and(
            () ->
                elevator.isStowed()
                    && hopper.atGoal(HopperState.FOLD)
                    && currentMode == DriveMode.CLIMB)
        .onTrue(
            Commands.sequence(
                climber.zeroOffset(),
                Commands.either(
                        climber.setStateOk(ClimberState.ALIGN),
                        Commands.none(),
                        () -> !climber.atGoal(ClimberState.ALIGN))
                    .alongWith(Commands.runOnce(() -> currentMode = DriveMode.TELEOP))
                    .andThen(
                        hopper.setStateOk(HopperState.INTAKE),
                        climber.setState(ClimberState.IDLE))));
  }

  public Command setElevatorState(ElevatorState state) {
    Command c =
        Commands.sequence(
            arm.setState(ArmState.IDLE),
            Commands.waitUntil(() -> arm.atGoal(ArmState.IDLE)),
            elevator.setState(state));
    // c.addRequirements(elevator, arm);
    return c;
  }

  public Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> driverGamepad.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> driverGamepad.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  public void periodic() {
    SmartDashboard.putBoolean("L4 Now", useLevel4);
  }

  public void disblePeriodic() {
    drive.stop();
    elevator.stop();
    arm.stop();
    hopper.stop();
  }

  enum DriveMode {
    CLIMB,
    TELEOP
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
