// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.hopper;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class HopperConstants {
  public static final ServoMotorSubsystemConfig kHopperConfig = new ServoMotorSubsystemConfig();

  static {
    kHopperConfig.name = "Hopper";
    kHopperConfig.canMasterId = 49;
    kHopperConfig.canBus = "canivore";

    kHopperConfig.kMinPositionUnits = 0;
    kHopperConfig.kMaxPositionUnits = 40;
    kHopperConfig.unitToRotorRatio = 1;

    kHopperConfig.enableSlave = false;
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    kHopperConfig.fxConfig = configuration;
  }
}
