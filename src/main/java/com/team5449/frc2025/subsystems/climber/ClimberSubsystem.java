// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.climber;

import static edu.wpi.first.units.Units.Rotation;

import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.MotorInputsAutoLogged;
import com.team5449.lib.subsystems.ServoMotorSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @AutoLogOutput @Setter @Getter private ClimberState desiredState = ClimberState.IDLE;

  public ClimberSubsystem(final MotorIO io) {
    super(ClimberConstants.kClimberConfig, new MotorInputsAutoLogged(), io);
    setCurrentPositionAsZero();
    setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  public Angle getStateAngle() {
    return desiredState.goalSetpoint.get();
  }

  public Command setState(ClimberState state) {
    return Commands.runOnce(() -> setDesiredState(state));
  }

  @RequiredArgsConstructor
  public enum ClimberState {
    IDLE(() -> Rotation.of(5)),
    ALIGN(() -> Rotation.of(80)),
    CLIMB(() -> Rotation.of(30));

    public final Supplier<Angle> goalSetpoint;
  }
}
