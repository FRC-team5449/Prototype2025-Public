// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMaster;
  private final TalonFX elevatorSlave;
  private final PositionDutyCycle positionControl = new PositionDutyCycle(0);
  private final TorqueCurrentFOC currentOpenLoop = new TorqueCurrentFOC(0);

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
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 19;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.CurrentLimits.StatorCurrentLimit = 90;
    configuration.Slot0.kG = 0.025;
    configuration.Slot0.kP = 0.1;
    configuration.Slot0.kI = 0.005;
    elevatorMaster.getConfigurator().apply(configuration);

    elevatorMaster.setPosition(0);
  }

  public Command positionCommand(double newPositionRotation) {
    return Commands.runOnce(() -> positionRotation = Rotation.of(newPositionRotation), this);
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
    BaseStatusSignal.refreshAll(elevatorAcceleration, elevatorVelocity);

    if (!characterizing) {
      elevatorMaster.setControl(
          positionControl.withEnableFOC(true).withSlot(0).withPosition(positionRotation));
      elevatorSlave.setControl(new Follower(0, false));
    }
  }
}
