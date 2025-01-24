// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import static com.team5449.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.LoggedTunableGeneric;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMaster;
  private final TalonFX elevatorSlave;
  private final PositionDutyCycle positionControl = new PositionDutyCycle(0);
  private final TorqueCurrentFOC currentOpenLoop = new TorqueCurrentFOC(0);
  private static final LoggedTunableGeneric<Angle, AngleUnit> rotationTarget =
      new LoggedTunableGeneric<>("Elevator/Level1_Rotation", Rotation);

  @Setter private Goal goal = Goal.LEVEL_1;

  private final StatusSignal<AngularAcceleration> elevatorAcceleration;
  private final StatusSignal<AngularVelocity> elevatorVelocity;

  private Angle positionRotation = Rotation.of(0);
  private boolean characterizing = false;

  public Elevator() {
    elevatorMaster = new TalonFX(1, "canivore");
    elevatorSlave = new TalonFX(2, "canivore");

    elevatorAcceleration = elevatorMaster.getAcceleration();
    elevatorVelocity = elevatorMaster.getVelocity();

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 17;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.CurrentLimits.StatorCurrentLimit = 90;
    configuration.Slot0.kG = 0.025;
    configuration.Slot0.kP = 0.1;
    configuration.Slot0.kI = 0.005;
    tryUntilOk(5, () -> elevatorMaster.getConfigurator().apply(configuration));

    elevatorMaster.setPosition(0);
    elevatorSlave.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command positionCommand(Angle newPosition) {
    return Commands.runOnce(() -> positionRotation = newPosition, this);
  }

  public void liftUpElevator() {
    switch (goal) {
      case LEVEL_1:
        goal = Goal.LEVEL_2;
        break;
      case LEVEL_2:
        goal = Goal.LEVEL_3;
        break;
      case LEVEL_3:
        goal = Goal.LEVEL_4;
        break;
      case LEVEL_4:
        goal = Goal.LEVEL_4;
        break;
    }
  }

  public void lowerDownElevator() {
    switch (goal) {
      case LEVEL_1:
        goal = Goal.LEVEL_1;
        break;
      case LEVEL_2:
        goal = Goal.LEVEL_1;
        break;
      case LEVEL_3:
        goal = Goal.LEVEL_2;
        break;
      case LEVEL_4:
        goal = Goal.LEVEL_3;
        break;
    }
  }

  public void runCharacterization(double currentAmps) {
    elevatorMaster.setControl(currentOpenLoop.withOutput(currentAmps));
    elevatorSlave.setControl(new Follower(0, false));
    characterizing = true;
  }

  public void endCharacterization() {
    characterizing = false;
  }

  @AutoLogOutput(key = "Elevator/Acceleration")
  public double getAcceleration() {
    return elevatorAcceleration.getValue().in(RotationsPerSecondPerSecond);
  }

  @AutoLogOutput(key = "Elevator/Velocity")
  public double getVelocity() {
    return elevatorVelocity.getValue().in(RotationsPerSecond);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Current Goal", goal.toString());
    BaseStatusSignal.refreshAll(elevatorAcceleration, elevatorVelocity);

    // positionRotation = goal.targetRotation.get();
    if (!characterizing) {
      elevatorMaster.setControl(
          positionControl.withEnableFOC(true).withSlot(0).withPosition(positionRotation));
      elevatorSlave.setControl(new Follower(1, true));
    }
  }

  public enum Goal {
    LEVEL_1(() -> Rotation.of(2)),
    LEVEL_2(() -> Rotation.of(5)),
    LEVEL_3(() -> Rotation.of(10)),
    LEVEL_4(() -> Rotation.of(17));

    public final Supplier<Angle> targetRotation;

    Goal(Supplier<Angle> targetRotation) {
      this.targetRotation = targetRotation;
    }

    public String toString() {
      switch (this) {
        case LEVEL_1:
          return "Level 1";
        case LEVEL_2:
          return "Level 2";
        case LEVEL_3:
          return "Level 3";
        case LEVEL_4:
          return "Level 4";
        default:
          return "";
      }
    }
  }
}
