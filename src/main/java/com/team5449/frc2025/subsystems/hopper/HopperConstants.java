// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.hopper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class HopperConstants {
  public static final ServoMotorSubsystemConfig kHopperConfig = new ServoMotorSubsystemConfig();

  public static final double positionTolerance = 0.5;

  static {
    kHopperConfig.name = "Hopper";
    kHopperConfig.canMasterId = 49;
    kHopperConfig.canBus = "canivore";

    kHopperConfig.kMinPositionUnits = 0;
    kHopperConfig.kMaxPositionUnits = 39;
    kHopperConfig.unitToRotorRatio = 1;

    kHopperConfig.enableSlave = false;
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    configuration.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(70);
    configuration.Slot0 = new Slot0Configs().withKP(1.5);
    configuration.SoftwareLimitSwitch =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kHopperConfig.kMaxPositionUnits)
            .withReverseSoftLimitThreshold(kHopperConfig.kMinPositionUnits);
    configuration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(100)
            .withMotionMagicAcceleration(1000);
    kHopperConfig.fxConfig = configuration;
  }
}
