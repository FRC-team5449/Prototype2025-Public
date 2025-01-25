// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import static com.team5449.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private final TalonFX armTalon;
  private final CANcoder armCanCoder;

  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(Goal.IDLE.targetPosition.get());
  private final StatusSignal<Angle> armPosition;

  private Goal goal = Goal.IDLE;

  /** Creates a new Arm. */
  public Arm() {
    armTalon = new TalonFX(7, "rio");
    armCanCoder = new CANcoder(12, "rio");

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.MagnetOffset = -0.84497;
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    tryUntilOk(5, () -> armCanCoder.getConfigurator().apply(canCoderConfiguration));

    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfiguration.CurrentLimits.StatorCurrentLimit = 90;
    talonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
    talonConfiguration.Slot0 =
        new Slot0Configs()
            .withKP(30)
            .withKI(0.2)
            .withKG(0.43)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    talonConfiguration.Feedback =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(12)
            .withFeedbackRotorOffset(0)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(41.66667)
            .withSensorToMechanismRatio(1);
    talonConfiguration.SoftwareLimitSwitch =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.254)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(-0.05);
    talonConfiguration.MotionMagic =
        new MotionMagicConfigs().withMotionMagicCruiseVelocity(0.8).withMotionMagicAcceleration(5);

    tryUntilOk(5, () -> armTalon.getConfigurator().apply(talonConfiguration));

    armPosition = armTalon.getPosition();
  }

  public Command positionCommand(Goal goalPosition) {
    return Commands.runOnce(() -> goal = goalPosition, this);
  }

  @AutoLogOutput(key = "Arm/NotStowed")
  public boolean notStowed() {
    return !MathUtil.isNear(
        Goal.STOW.targetPosition.get().in(Rotation), armPosition.getValue().in(Rotation), 0.02);
  }

  public boolean isStowed() {
    return MathUtil.isNear(
        Goal.STOW.targetPosition.get().in(Rotation), armPosition.getValue().in(Rotation), 0.02);
  }

  public enum Goal {
    STOW(() -> Rotation.of(0.22)),
    IDLE(() -> Rotation.of(0.15)),
    SCORE(() -> Rotation.of(0.13)),
    ;
    private final Supplier<Angle> targetPosition;

    Goal(Supplier<Angle> targetPosition) {
      this.targetPosition = targetPosition;
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(armPosition);
    armTalon.setControl(positionControl.withSlot(0).withPosition(goal.targetPosition.get()));
  }
}
