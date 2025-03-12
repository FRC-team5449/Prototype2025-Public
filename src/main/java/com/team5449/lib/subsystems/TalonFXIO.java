// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import static com.team5449.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.frc2025.Robot;
import com.team5449.lib.util.UnitUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {
  protected final TalonFX talon;
  protected TalonFX talonSlave;
  protected final ServoMotorSubsystemConfig mConfig;

  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(RPM.of(0));
  private final PositionVoltage positionVoltageControl = new PositionVoltage(Radian.of(0));
  private final MotionMagicVoltage motionMagicVoltageControl = new MotionMagicVoltage(Radian.of(0));

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
    maxPosition = Rotation.of(mConfig.kMaxPositionUnits);
    minPosition = Rotation.of(mConfig.kMinPositionUnits);

    talon = new TalonFX(mConfig.canMasterId, mConfig.canBus);

    if (Robot.isSimulation()) {
      mConfig.fxConfig.CurrentLimits = new CurrentLimitsConfigs();
      mConfig.fxConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
      mConfig.fxConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
    }

    tryUntilOk(5, () -> talon.getConfigurator().apply(mConfig.fxConfig, 0.25));

    if (config.enableSlave) {
      talonSlave = new TalonFX(config.canSlaveId, config.canBus);
      tryUntilOk(5, () -> talonSlave.getConfigurator().apply(config.fxSlaveConfig));
      tryUntilOk(5, () -> talonSlave.optimizeBusUtilization());
    }

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

    velocityVoltageControl.UpdateFreqHz = 50.0;
    positionVoltageControl.UpdateFreqHz = 50.0;
    motionMagicVoltageControl.UpdateFreqHz = 50.0;
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(signals).equals(StatusCode.OK);
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    inputs.positionRotation = positionSignal.getValueAsDouble();
    inputs.velocityRotationPerSec = velocitySignal.getValueAsDouble();
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(new DutyCycleOut(dutyCycle));
    setSlaveMotorFollowing();
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    Angle setPosition = UnitUtil.clamp(position, minPosition, maxPosition);

    talon.setControl(positionVoltageControl.withPosition(setPosition));
    setSlaveMotorFollowing();
  }

  @Override
  public void setMotionMagicSetpoint(Angle position) {
    Angle setPosition = UnitUtil.clamp(position, minPosition, maxPosition);
    talon.setControl(motionMagicVoltageControl.withPosition(setPosition));
    setSlaveMotorFollowing();
  }

  @Override
  public void setMotionMagicSetpoint(double positionRotation) {
    positionRotation =
        MathUtil.clamp(positionRotation, mConfig.kMinPositionUnits, mConfig.kMaxPositionUnits);
    talon.setControl(motionMagicVoltageControl.withPosition(positionRotation));
    setSlaveMotorFollowing();
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
    talon.setControl(velocityVoltageControl.withVelocity(velocity));
    setSlaveMotorFollowing();
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(Radian.of(0));
  }

  @Override
  public void setCurrentPosition(Angle position) {
    talon.setPosition(position);
  }

  public void setSlaveMotorFollowing() {
    if (talonSlave != null && mConfig.enableSlave) {
      talonSlave.setControl(new Follower(mConfig.canMasterId, mConfig.isSlaveOpposite));
    }
  }

  @Override
  public void setEnableSlaveMotor(boolean enableSlave) {
    mConfig.enableSlave = enableSlave;
  }

  @Override
  public void runCharacterization(double currentAmps) {
    talon.setControl(new TorqueCurrentFOC(currentAmps));
  }

  @Override
  public void stop() {
    talon.set(0);
    setSlaveMotorFollowing();
  }
}
