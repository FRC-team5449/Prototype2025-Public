// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class ElevatorConstants {
  public static final ServoMotorSubsystemConfig kElevatorConfig = new ServoMotorSubsystemConfig();

  static {
    kElevatorConfig.name = "Elevator";
    kElevatorConfig.canBus = "canivore";
    kElevatorConfig.canId = 0;
    kElevatorConfig.fxConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(90))
            .withSlot0(new Slot0Configs().withKP(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
    kElevatorConfig.kMaxPositionUnits = 300;
    kElevatorConfig.kMinPositionUnits = -300;
    kElevatorConfig.unitToRotorRatio = 1;
  }
}
