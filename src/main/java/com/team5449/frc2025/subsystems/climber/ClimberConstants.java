// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class ClimberConstants {
  public static final ServoMotorSubsystemConfig kClimberConfig = new ServoMotorSubsystemConfig();

  static {
    kClimberConfig.name = "Climber";
    kClimberConfig.canBus = "canivore";
    kClimberConfig.canMasterId = 42;
    kClimberConfig.canSlaveId = 41;
    kClimberConfig.enableSlave = true;
    kClimberConfig.isSlaveOpposite = true;
    kClimberConfig.kMinPositionUnits = 5;
    kClimberConfig.kMaxPositionUnits = 80;
    kClimberConfig.unitToRotorRatio = 1;

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    configuration.Slot0 =
        new Slot0Configs().withKP(0.5).withGravityType(GravityTypeValue.Elevator_Static);
    configuration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1000)
            .withMotionMagicAcceleration(1600);
    kClimberConfig.fxConfig = configuration;
    kClimberConfig.fxSlaveConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }
}
