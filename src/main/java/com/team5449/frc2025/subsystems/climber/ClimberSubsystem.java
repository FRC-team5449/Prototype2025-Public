// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.climber;

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

public class ClimberSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @Setter @Getter private ClimberState desiredState = ClimberState.IDLE;

  private double climbOffset = 0;

  public ClimberSubsystem(final MotorIO io) {
    super(ClimberConstants.kClimberConfig, new MotorInputsAutoLogged(), io);
    setCurrentPositionAsZero();
    setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  public double getStateAngle() {
    if (desiredState == ClimberState.ALIGN) {
      return desiredState.goalSetpoint.getAsDouble() + climbOffset;
    }
    return desiredState.goalSetpoint.getAsDouble();
  }

  public Command setState(ClimberState state) {
    return Commands.runOnce(() -> setDesiredState(state));
  }

  public Command decline() {
    return Commands.runEnd(
        () -> {
          climbOffset += 1;
        },
        () -> {});
  }

  public Command zeroOffset() {
    return Commands.runOnce(() -> climbOffset = 0);
  }

  public Command elevate() {
    return Commands.runEnd(
        () -> {
          climbOffset -= 1;
        },
        () -> {});
  }

  public boolean atGoal() {
    return atGoal(desiredState);
  }

  public Command setStateOk(ClimberState state) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> this.atGoal(state)), setState(state));
  }

  @AutoLogOutput
  public boolean atGoal(ClimberState setState) {
    return MathUtil.isNear(
        inputs.positionRotation,
        setState.goalSetpoint.getAsDouble(),
        ClimberConstants.positionTolerance);
  }

  @RequiredArgsConstructor
  public enum ClimberState {
    IDLE(() -> 0),
    ALIGN(() -> 53);

    public final DoubleSupplier goalSetpoint;
  }
}
