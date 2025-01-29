// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import static com.team5449.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.ServoMotorSubsystemConfig;
import com.team5449.lib.util.UnitUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmTalonIO implements MotorIO {
  protected final CANcoder armCancoder;
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig mConfig = ArmConstants.kArmConfig;

  private final MotionMagicExpoVoltage motionMagicVoltageControl =
      new MotionMagicExpoVoltage(Radian.of(0));

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  private final Angle maxPosition;
  private final Angle minPosition;

  private BaseStatusSignal[] signals;

  public ArmTalonIO() {
    maxPosition = Rotation.of(mConfig.kMaxPositionUnits);
    minPosition = Rotation.of(mConfig.kMinPositionUnits);

    talon = new TalonFX(mConfig.canMasterId, mConfig.canBus);

    armCancoder = new CANcoder(ArmConstants.armCanCoderId, ArmConstants.armCanCoderBus);

    tryUntilOk(5, () -> talon.getConfigurator().apply(mConfig.fxConfig, 0.25));
    tryUntilOk(5, () -> armCancoder.getConfigurator().apply(ArmConstants.armCanCoderConfig, 0.25));

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
        };
    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals));
    tryUntilOk(5, () -> talon.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(signals).equals(StatusCode.OK);
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    inputs.position = positionSignal.getValue();
    inputs.velocity = velocitySignal.getValue();
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(new DutyCycleOut(dutyCycle));
  }

  @Override
  public void setMotionMagicSetpoint(Angle position) {
    Angle setPosition = UnitUtil.clamp(position, minPosition, maxPosition);
    talon.setControl(motionMagicVoltageControl.withPosition(setPosition));
  }

  @Override
  public void setEnableSoftLimits(boolean forward, boolean reverse) {
    mConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    mConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = forward;
    tryUntilOk(5, () -> talon.getConfigurator().apply(mConfig.fxConfig));
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    talon.setNeutralMode(mode);
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(Radian.of(0));
  }

  @Override
  public void setCurrentPosition(Angle position) {
    talon.setPosition(position);
  }

  @Override
  public void runCharacterization(double currentAmps) {
    talon.setControl(new TorqueCurrentFOC(currentAmps));
  }
}
