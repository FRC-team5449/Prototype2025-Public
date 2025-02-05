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
import com.team5449.lib.util.UnitUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @Setter private ArmState desiredState = ArmState.IDLE;

  public ArmSubsystem(final MotorIO io) {
    super(ArmConstants.kArmConfig, new MotorInputsAutoLogged(), io);
    setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  // public Command motionMagicSetpointCommand(Supplier<ArmState> stateSupplier) {
  //   return motionMagicSetpointCommand(stateSupplier);
  // }
  public Angle getStateAngle() {
    return desiredState.goalSetpoint.get();
  }

  public Command setStateCommand(ArmState state) {
    return Commands.runOnce(() -> this.setDesiredState(state));
  }

  @AutoLogOutput(key = "Arm/isStowed")
  public boolean isStowed() {
    return atGoal(ArmState.STOW);
  }

  public boolean atGoal() {
    return atGoal(desiredState);
  }

  public boolean atGoal(ArmState setState) {
    return UnitUtil.isNear(
        inputs.position, setState.goalSetpoint.get(), ArmConstants.positionTolerance);
  }

  public enum ArmState {
    IDLE(() -> Rotation.of(0.15)),
    STOW(() -> Rotation.of(0.22)),
    SCORE(() -> Rotation.of(0.13));

    public final Supplier<Angle> goalSetpoint;

    ArmState(Supplier<Angle> goalSetpoint) {
      this.goalSetpoint = goalSetpoint;
    }
  }
}
