// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.servo;

import static com.team5449.lib.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team5449.frc2025.Robot;
import com.team5449.lib.UnitUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig mConfig;
  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(AngularVelocity.ofBaseUnits(0, RPM));
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(Angle.ofBaseUnits(0, Radian));
  private final MotionMagicExpoTorqueCurrentFOC motionMagicControl =
      new MotionMagicExpoTorqueCurrentFOC(Angle.ofBaseUnits(0, Radian));
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
    maxPosition = Angle.ofBaseUnits(mConfig.kMaxPositionUnits, Radian);
    minPosition = Angle.ofBaseUnits(mConfig.kMinPositionUnits, Radian);

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

  public double clampPosition(double units) {
    return unitsToRotor(
        MathUtil.clamp(units, mConfig.kMinPositionUnits, mConfig.kMaxPositionUnits));
  }

  private double rotorToUnits(double rotor) {
    return rotor * mConfig.unitToRotorRatio;
  }

  public double unitsToRotor(double units) {
    return units / mConfig.unitToRotorRatio;
  }

  @Override
  public void processInputs(MotorInputs inputs) {
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
  public void setPositionSetpoint(Angle units) {
    talon.setControl(positionControl.withPosition(UnitUtil.clamp(units, minPosition, maxPosition)));
  }
}
