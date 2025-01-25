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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team5449.lib.LoggedTunableGeneric;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMaster;
  private final TalonFX elevatorSlave;
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
  private final TorqueCurrentFOC currentOpenLoop = new TorqueCurrentFOC(0);
  private static final LoggedTunableGeneric<Angle, AngleUnit> rotationTarget =
      new LoggedTunableGeneric<>("Elevator/Level1_Rotation", Rotation);
  private final double positionTolerance = 0.3;

  @Getter private Goal goal = Goal.IDLE;

  private final StatusSignal<AngularAcceleration> elevatorAcceleration;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Angle> elevatorPosition;

  private Angle setpointPosition = Rotation.of(0);
  private boolean characterizing = false;
  private boolean openLoopControl = false;

  @AutoLogOutput(key = "Elevator/Homing")
  private boolean isHoming = false;

  private final Debouncer homingDelay = new Debouncer(0.2);

  public Elevator() {
    elevatorMaster = new TalonFX(1, "canivore");
    elevatorSlave = new TalonFX(2, "canivore");

    elevatorAcceleration = elevatorMaster.getAcceleration();
    elevatorVelocity = elevatorMaster.getVelocity();
    elevatorPosition = elevatorMaster.getPosition();

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 17.8;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configuration.CurrentLimits.StatorCurrentLimit = 70;

    configuration.Slot0.kP = 1.7;
    configuration.Slot0.kI = 0;
    configuration.Slot0.kD = 0.04;
    configuration.Slot0.kS = 0.12;
    configuration.Slot0.kG = 0.33;
    configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    configuration.MotionMagic.MotionMagicCruiseVelocity = 35;
    configuration.MotionMagic.MotionMagicAcceleration = 300;
    configuration.MotionMagic.MotionMagicJerk = 0;
    configuration.MotionMagic.MotionMagicExpo_kV = 0.19;
    configuration.MotionMagic.MotionMagicExpo_kA = 0.3;
    tryUntilOk(5, () -> elevatorMaster.getConfigurator().apply(configuration));

    elevatorMaster.setPosition(0);
    elevatorSlave.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCurrentPosition(Angle setPosition) {
    elevatorMaster.setPosition(setPosition);
  }

  public void setCurrentAsZero() {
    setCurrentPosition(Rotation.of(0));
  }

  private boolean atHomingLocation() {
    return MathUtil.isNear(0, elevatorPosition.getValue().in(Rotation), positionTolerance);
  }

  private void updateHoming() {
    // Auto-start homing when at home position
    if (goal == Goal.IDLE && atHomingLocation() && !isHoming) {
      // Start Homing
      isHoming = true;
    }

    if (isHoming) {
      runOpenLoop(-0.4); // Apply homing voltage
      if (homingDelay.calculate(
          MathUtil.isNear(0, elevatorVelocity.getValue().in(RotationsPerSecond), 0.05))) {
        // Stop Homing
        isHoming = false;
        setCurrentAsZero();
        runSetpoint(Goal.IDLE.targetRotation.get());
      }
    }
  }

  public void runOpenLoop(double volts) {
    openLoopControl = true;
    elevatorMaster.setControl(new VoltageOut(volts));
    elevatorSlave.setControl(new Follower(1, true));
  }

  public void runSetpoint(Angle setpointPosition) {
    openLoopControl = false;
    this.setpointPosition = setpointPosition;
  }

  public Command positionCommand(Angle newPosition) {
    return Commands.runOnce(() -> runSetpoint(newPosition), this);
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

  public boolean isStowed() {
    return MathUtil.isNear(0, elevatorPosition.getValue().in(Rotation), 0.3);
  }

  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean atGoal() {
    return !isStowed()
        && MathUtil.isNear(
            setpointPosition.in(Rotation),
            elevatorPosition.getValue().in(Rotation),
            positionTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Current Goal", goal.toString());
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorAcceleration, elevatorVelocity);

    // updateHoming();

    Logger.recordOutput("Elevator/SetpointPosition", setpointPosition.in(Rotation));
    Logger.recordOutput("Elevator/CurrentPosition", elevatorPosition.getValue().in(Rotation));

    if (!characterizing && !isHoming && !openLoopControl) {
      elevatorMaster.setControl(
          motionMagicControl.withEnableFOC(true).withSlot(0).withPosition(setpointPosition));
      elevatorSlave.setControl(new Follower(1, true));
    }
  }

  public enum Goal {
    IDLE(() -> Rotation.of(0)),
    LEVEL_1(() -> Rotation.of(2)),
    LEVEL_2(() -> Rotation.of(5)),
    LEVEL_3(() -> Rotation.of(10)),
    LEVEL_4(() -> Rotation.of(17.5));

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
