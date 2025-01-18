// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMaster = new TalonFX(1, "canivore");
  private final TalonFX elevatorSlave = new TalonFX(2, "canivore");
  private final PositionDutyCycle positionControl = new PositionDutyCycle(-0.03);
  private double positionRotation = 0;

  public Elevator() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.CurrentLimits.StatorCurrentLimit = 90;
    configuration.Slot0.kG = 0.025;
    configuration.Slot0.kP = 0.1;
    configuration.Slot0.kI = 0.005;
    elevatorMaster.getConfigurator().apply(configuration);
  }

  public Command positionCommand(double newPositionRotation) {
    return Commands.runOnce(() -> positionRotation = newPositionRotation, this);
  }

  @Override
  public void periodic() {
    elevatorMaster.setControl(
        positionControl.withEnableFOC(true).withSlot(0).withPosition(positionRotation));
    elevatorSlave.setControl(new Follower(0, false));
  }
}
