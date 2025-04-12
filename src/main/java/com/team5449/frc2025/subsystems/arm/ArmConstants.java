// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;

public class ArmConstants {
  public static final ServoMotorSubsystemConfig kArmConfig = new ServoMotorSubsystemConfig();
  public static final double positionTolerance = 0.02;

  public static final double kArmGearRatio = 62.5;

  public static final CANcoderConfiguration armCanCoderConfig = new CANcoderConfiguration();
  public static final int armCanCoderId = 8;
  public static final String armCanCoderBus = "rio";

  static {
    kArmConfig.name = "Arm";
    kArmConfig.canMasterId = 7;
    kArmConfig.canBus = "rio";

    kArmConfig.enableSlave = false;

    kArmConfig.kMaxPositionUnits = 0.295;
    kArmConfig.kMinPositionUnits = -0.05;

    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    talonConfiguration.SoftwareLimitSwitch =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kArmConfig.kMaxPositionUnits)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kArmConfig.kMinPositionUnits);
    talonConfiguration.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(90)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(70);
    talonConfiguration.Slot0 =
        new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(1)
            .withKS(0.23)
            .withKG(0.44)
            .withKV(0.06)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    talonConfiguration.Feedback =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(armCanCoderId)
            .withFeedbackRotorOffset(0)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(kArmGearRatio)
            .withSensorToMechanismRatio(1);
    talonConfiguration.MotionMagic =
        new MotionMagicConfigs().withMotionMagicCruiseVelocity(30).withMotionMagicAcceleration(30);

    kArmConfig.fxConfig = talonConfiguration;
  }

  static {
    armCanCoderConfig.MagnetSensor.MagnetOffset = 0.11;
    armCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    armCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
  }
}
