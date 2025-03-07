// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

import static edu.wpi.first.units.Units.Rotation;

import com.team5449.lib.subsystems.MotorIO;
import com.team5449.lib.subsystems.MotorInputsAutoLogged;
import com.team5449.lib.subsystems.ServoMotorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @Setter @Getter private ArmState desiredState = ArmState.IDLE;

  public ArmSubsystem(final MotorIO io) {
    super(ArmConstants.kArmConfig, new MotorInputsAutoLogged(), io);
    setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  private double getStateAngle() {
    return desiredState.goalSetpoint.getAsDouble();
  }

  public Command setStateCommand(ArmState state) {
    return Commands.runOnce(() -> this.setDesiredState(state));
  }

  @AutoLogOutput(key = "Arm/isIdle")
  public boolean idle() {
    return atGoal(ArmState.IDLE);
  }

  public boolean intaking() {
    return atGoal(ArmState.INTAKE);
  }

  public boolean atGoal() {
    return atGoal(desiredState);
  }

  public boolean atGoal(ArmState setState) {
    return MathUtil.isNear(
        inputs.position.in(Rotation),
        setState.goalSetpoint.getAsDouble(),
        ArmConstants.positionTolerance);
  }

  @RequiredArgsConstructor
  public enum ArmState {
    IDLE(() -> 0.182891),
    INTAKE(() -> 0.255),
    SCORE(() -> 0.092891);

    public final DoubleSupplier goalSetpoint;
  }
}
