// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class ArmSimTalonIO extends ArmTalonIO {
  private final DCMotor armPlant = DCMotor.getFalcon500(1);
  private final DCMotorSim mechanismSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(armPlant, 0.1, 1.0 / ArmConstants.kArmGearRatio),
          armPlant,
          0.2981858,
          .2981858);

  private double lastUpdateTimestamp;
  private Notifier simNotifier = null;

  public ArmSimTalonIO() {
    armCancoder.setPosition(0);

    simNotifier =
        new Notifier(
            () -> {

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState();
            });

    simNotifier.startPeriodic(0.005);
    armCancoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    super.updateInputs(inputs);
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

  public void updateSimState() {
    var simState = talon.getSimState();
    simState.setSupplyVoltage(12.0);
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

    mechanismSim.setInput(simVoltage);
    Logger.recordOutput("Arm/Sim/SimulatorVoltage", simVoltage);

    double timestamp = Timer.getTimestamp();
    mechanismSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    Angle simArmPosition = mechanismSim.getAngularPosition();
    AngularVelocity simArmVelocity = mechanismSim.getAngularVelocity();
    Logger.recordOutput("Arm/Sim/SimulatorPositionRadians", simArmPosition.in(Radian));

    armCancoder.getSimState().setRawPosition(simArmPosition.in(Rotation));
    armCancoder.getSimState().setVelocity(mechanismSim.getAngularVelocity().in(RotationsPerSecond));
    double simArmCancoderPosition = armCancoder.getPosition().getValueAsDouble();
    Logger.recordOutput("Arm/Sim/CANcoder Position", simArmCancoderPosition);

    double rotorPosition = simArmPosition.in(Rotation) / ArmConstants.kArmGearRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Arm/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    double rotorVel = simArmVelocity.in(RadiansPerSecond) / ArmConstants.kArmGearRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        "Turret/Sim/SimulatorVelocityRadS", mechanismSim.getAngularVelocityRadPerSec());
  }
}
