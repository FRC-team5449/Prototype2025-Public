// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class ElevatorConstants {
  public static final ServoMotorSubsystemConfig kElevatorConfig = new ServoMotorSubsystemConfig();
  public static final double positionTolerance = 0.3;

  static {
    kElevatorConfig.name = "Elevator";
    kElevatorConfig.canBus = "canivore";
    kElevatorConfig.canMasterId = 1;

    kElevatorConfig.enableSlave = true;
    kElevatorConfig.canSlaveId = 2;
    kElevatorConfig.isSlaveOpposite = true;

    kElevatorConfig.kMaxPositionUnits = 21 * (3.0 / 4.0);
    kElevatorConfig.kMinPositionUnits = -0.05;

    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    talonConfiguration.SoftwareLimitSwitch =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kElevatorConfig.kMaxPositionUnits)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kElevatorConfig.kMinPositionUnits);
    talonConfiguration.CurrentLimits.StatorCurrentLimit = 120;
    talonConfiguration.Slot0 =
        new Slot0Configs()
            .withKP(1.85)
            .withKI(0)
            .withKD(0.1)
            .withKS(0)
            .withKG(0.35)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    talonConfiguration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicAcceleration(1000)
            .withMotionMagicExpo_kV(0.19)
            .withMotionMagicExpo_kA(0.3);

    kElevatorConfig.fxConfig = talonConfiguration;

    kElevatorConfig.fxSlaveConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    kElevatorConfig.unitToRotorRatio = 1;
  }
}
