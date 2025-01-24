// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import static com.team5449.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX armTalon;
  private final PositionDutyCycle positionControl = new PositionDutyCycle(0);
  private Angle positionRotation = Rotation.of(5.3);

  /** Creates a new Arm. */
  public Arm() {
    armTalon = new TalonFX(7, "rio");
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.4;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 2.3;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.CurrentLimits.StatorCurrentLimit = 90;
    configuration.Slot0.kG = 0.05;
    configuration.Slot0.kP = 0.1;
    configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configuration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // configuration.Slot0.kI = 0.005;

    tryUntilOk(5, () -> armTalon.getConfigurator().apply(configuration));
  }

  public Command positionCommand(Angle newPosition) {
    return Commands.runOnce(() -> positionRotation = newPosition, this);
  }

  @Override
  public void periodic() {
    armTalon.setControl(
        positionControl.withEnableFOC(true).withSlot(0).withPosition(positionRotation));
  }
}
