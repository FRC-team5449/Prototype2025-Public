// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private double lastUpdateTimestamp = 0.0;

  public SimTalonFXIO(ServoMotorSubsystemConfig config) {
    super(config);

    sim = null;
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), mConfig.momentOfInertia, 1.0 / mConfig.unitToRotorRatio),
            DCMotor.getKrakenX60(1));
    // Assume that config is correct (which it might not be)
    talon.getSimState().Orientation =
        (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  protected void updateSimState() {
    var simState = talon.getSimState();
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

    sim.setInput(simVoltage);
    Logger.recordOutput(mConfig.name + "/Sim/SimulatorVoltage", simVoltage);

    double timestamp = Timer.getFPGATimestamp();
    sim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    // Find current state of sim in radians from 0 point
    double simPositionRads = sim.getAngularPositionRad();
    Logger.recordOutput(mConfig.name + "/Sim/SimulatorPositionRadians", simPositionRads);

    // Mutate rotor position
    double rotorPosition = Units.radiansToRotations(simPositionRads) / mConfig.unitToRotorRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput(mConfig.name + "/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) / mConfig.unitToRotorRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        mConfig.name + "/Sim/SimulatorVelocityRadS", sim.getAngularVelocityRadPerSec());
  }
}
