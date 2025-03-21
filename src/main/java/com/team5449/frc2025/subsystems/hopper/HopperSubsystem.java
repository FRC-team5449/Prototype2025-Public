// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.hopper;

import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.MotorInputsAutoLogged;
import com.team5449.lib.subsystems.ServoMotorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class HopperSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @Setter @Getter private HopperState desiredState = HopperState.INTAKE;

  public HopperSubsystem(final MotorIO io) {
    super(HopperConstants.kHopperConfig, new MotorInputsAutoLogged(), io);
    setCurrentPositionAsZero();
    // setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  @Override
  public void periodic() {
    super.periodic();
    setMotionMagicSetpointImpl(desiredState.goalSetpoint);
  }

  public double getStateAngle() {
    return desiredState.goalSetpoint.getAsDouble();
  }

  public Command decline() {
    return startEnd(() -> io.setOpenLoopDutyCycle(0.2), () -> io.setOpenLoopDutyCycle(0));
  }

  public Command elevate() {
    return startEnd(() -> io.setOpenLoopDutyCycle(-0.2), () -> io.setOpenLoopDutyCycle(0));
  }

  public Command setState(HopperState state) {
    return Commands.runOnce(() -> setDesiredState(state));
  }

  public boolean atGoal() {
    return atGoal(desiredState);
  }

  public Command setStateOk(HopperState state) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> this.atGoal(state)), setState(state));
  }

  public boolean atGoal(HopperState setState) {
    return MathUtil.isNear(
        inputs.positionRotation,
        setState.goalSetpoint.getAsDouble(),
        HopperConstants.positionTolerance);
  }

  @RequiredArgsConstructor
  public enum HopperState {
    IDLE(() -> 0),
    INTAKE(() -> 2),
    FOLD(() -> 39);

    public final DoubleSupplier goalSetpoint;
  }
}
