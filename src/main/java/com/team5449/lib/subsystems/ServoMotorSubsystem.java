// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
    extends SubsystemBase {
  protected U io;
  protected T inputs;
  private Angle positionSetpoint = Radian.of(0);

  protected ServoMotorSubsystemConfig config;

  public ServoMotorSubsystem(ServoMotorSubsystemConfig config, T inputs, U io) {
    super(config.name);
    this.config = config;
    this.io = io;
    this.inputs = inputs;

    // setDefaultCommand((() -> 0.0).withName(
    //   getName() + " Default Command Neutral"));
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput(getName() + "/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Angle getCurrentPosition() {
    return inputs.position;
  }

  public AngularVelocity getCurrentVelocity() {
    return inputs.velocity;
  }

  public Angle getPositionSetpoint() {
    return positionSetpoint;
  }

  protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
    Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
    io.setOpenLoopDutyCycle(dutyCycle);
  }

  protected void setPositionSetpointImpl(Angle position) {
    positionSetpoint = position;
    Logger.recordOutput(getName() + "/API/setPositionSetpointImp/Units", position);
    io.setPositionSetpoint(position);
  }

  protected void setMotionMagicSetpointImpl(Supplier<Angle> positionSupplier) {
    positionSetpoint = positionSupplier.get();
    Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Units", positionSetpoint);
    io.setMotionMagicSetpoint(positionSetpoint);
  }

  protected void setVelocitySetpointImpl(AngularVelocity velocity) {
    Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS", velocity);
    io.setVelocitySetpoint(velocity);
  }

  protected void setNeutralModeImpl(NeutralModeValue mode) {
    Logger.recordOutput(getName() + "/API/setNeutralModeImpl/Mode", mode);
    io.setNeutralMode(mode);
  }

  public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {
              setOpenLoopDutyCycleImpl(0.0);
            })
        .withName(getName() + " DutyCycleControl");
  }

  public Command setCoast() {
    return startEnd(
            () -> setNeutralModeImpl(NeutralModeValue.Coast),
            () -> setNeutralModeImpl(NeutralModeValue.Brake))
        .withName(getName() + "CoastMode")
        .ignoringDisable(true);
  }

  public Command positionSetpointCommand(Supplier<Angle> positionSupplier) {
    return runEnd(
            () -> {
              setPositionSetpointImpl(positionSupplier.get());
            },
            () -> {})
        .withName(getName() + " positionSetpointCommand");
  }

  public Command motionMagicSetpointCommand(Supplier<Angle> positionSupplier) {
    return runEnd(
            () -> {
              setMotionMagicSetpointImpl(positionSupplier);
            },
            () -> {})
        .withName(getName() + " motionMagicSetpointCommand");
  }

  public Command positionSetpointUntilOnTargetCommand(
      Supplier<Angle> positionSupplier, DoubleSupplier epsilon) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(
            () ->
                MathUtil.isNear(
                    positionSupplier.get().in(Radian),
                    inputs.position.in(Radian),
                    epsilon.getAsDouble())),
        positionSetpointCommand(positionSupplier));
  }

  protected void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  protected void setCurrentPosition(Angle position) {
    io.setCurrentPosition(position);
  }
}
