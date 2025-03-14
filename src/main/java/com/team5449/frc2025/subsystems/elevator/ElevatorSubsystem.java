// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

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
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @AutoLogOutput @Setter @Getter private ElevatorState desiredState = ElevatorState.IDLE;

  public ElevatorSubsystem(final MotorIO io) {
    super(ElevatorConstants.kElevatorConfig, new MotorInputsAutoLogged(), io);
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

  public Command setState(ElevatorState state) {
    return Commands.runOnce(() -> setDesiredState(state));
  }

  @AutoLogOutput
  public boolean isStowed() {
    return atGoal(ElevatorState.IDLE);
  }

  @AutoLogOutput
  public boolean atGoal() {
    return atGoal(desiredState);
  }

  public boolean atGoal(ElevatorState setState) {
    return MathUtil.isNear(
        inputs.positionRotation,
        setState.goalSetpoint.getAsDouble(),
        ElevatorConstants.positionTolerance);
  }

  public Command setStateOk(ElevatorState state) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> this.atGoal(state)), setState(state));
  }

  @RequiredArgsConstructor
  public enum ElevatorState {
    IDLE(() -> 0),
    L1(() -> 1.2),
    L2(() -> 4.3 * (5.0 / 4.0)),
    L3(() -> 10 * (5.0 / 4.0)),
    L4(() -> 20 * (5.0 / 4.0));

    public final DoubleSupplier goalSetpoint;
  }

  // Check for initialization delays or homing sequence
}
