// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import static com.team5449.lib.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.frc2025.Robot;
import com.team5449.lib.UnitUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig mConfig;
  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(RPM.of(0));
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(Radian.of(0));
  private final MotionMagicExpoTorqueCurrentFOC motionMagicControl =
      new MotionMagicExpoTorqueCurrentFOC(Radian.of(0));
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  private final Angle maxPosition;
  private final Angle minPosition;

  private BaseStatusSignal[] signals;

  public TalonFXIO(ServoMotorSubsystemConfig config) {
    mConfig = config;
    maxPosition = Radian.of(mConfig.kMaxPositionUnits);
    minPosition = Radian.of(mConfig.kMinPositionUnits);

    talon = new TalonFX(config.canId, config.canBus);

    if (Robot.isSimulation()) {
      mConfig.fxConfig.CurrentLimits = new CurrentLimitsConfigs();
      mConfig.fxConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
      mConfig.fxConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
    }

    tryUntilOk(5, () -> talon.getConfigurator().apply(mConfig.fxConfig, 0.25));

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
    talon.setControl(dutyCycleControl.withOutput(dutyCycle));
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    talon.setControl(
        positionControl.withPosition(UnitUtil.clamp(position, minPosition, maxPosition)));
  }

  @Override
  public void setMotionMagicSetpoint(Angle position) {
    talon.setControl(
        motionMagicControl.withPosition(UnitUtil.clamp(position, minPosition, maxPosition)));
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
  public void setVelocitySetpoint(AngularVelocity velocity) {
    talon.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(Radian.of(0));
  }

  @Override
  public void setCurrentPosition(Angle position) {
    talon.setPosition(position);
  }
}
