// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotation;

import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.MotorInputsAutoLogged;
import com.team5449.lib.subsystems.ServoMotorSubsystem;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;
import com.team5449.lib.subsystems.TalonFXIO.ControlType;

public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public ElevatorSubsystem(ServoMotorSubsystemConfig config, final MotorIO io) {
    super(config, new MotorInputsAutoLogged(), io);
    io.setControlType(ControlType.DUTY_CYCLE);
    setCurrentPositionAsZero();
    setDefaultCommand(positionSetpointCommand(() -> Rotation.of(0)));
  }
}
