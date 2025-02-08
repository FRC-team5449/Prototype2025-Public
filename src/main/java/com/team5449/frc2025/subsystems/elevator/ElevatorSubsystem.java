// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotation;

import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.MotorInputsAutoLogged;
import com.team5449.lib.subsystems.ServoMotorSubsystem;
import com.team5449.lib.util.UnitUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @AutoLogOutput @Setter private ElevatorState desiredState = ElevatorState.IDLE;

  public ElevatorSubsystem(final MotorIO io) {
    super(ElevatorConstants.kElevatorConfig, new MotorInputsAutoLogged(), io);
    setCurrentPositionAsZero();
    setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  public Angle getStateAngle() {
    return desiredState.goalSetpoint.get();
  }

  public Command setStateCommand(ElevatorState state) {
    return Commands.runOnce(() -> this.setDesiredState(state));
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
    return UnitUtil.isNear(
        inputs.position, setState.goalSetpoint.get(), ElevatorConstants.positionTolerance);
  }

  public Command autoSetStateCommand(ElevatorState state) {
    return Commands.sequence(setStateCommand(state), new WaitUntilCommand(() -> atGoal()));
  }

  @RequiredArgsConstructor
  public enum ElevatorState {
    IDLE(() -> Rotation.of(0)),
    LEVEL_1(() -> Rotation.of(2)),
    LEVEL_2(() -> Rotation.of(5)),
    LEVEL_3(() -> Rotation.of(11.2)),
    LEVEL_4(() -> Rotation.of(17.5));

    public final Supplier<Angle> goalSetpoint;
  }
}
