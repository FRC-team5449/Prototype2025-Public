// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.arm;

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

public class ArmSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  @AutoLogOutput @Setter @Getter private ArmState desiredState = ArmState.INTAKE;

  public ArmSubsystem(final MotorIO io) {
    super(ArmConstants.kArmConfig, new MotorInputsAutoLogged(), io);
    // setDefaultCommand(motionMagicSetpointCommand(this::getStateAngle));
  }

  @Override
  public void periodic() {
    super.periodic();
    setMotionMagicSetpointImpl(desiredState.goalSetpoint);
  }

  public Command setState(ArmState state) {
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

  public Command setStateOk(ArmState state) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> this.atGoal(state)), setState(state));
  }

  public boolean atGoal(ArmState setState) {
    return MathUtil.isNear(
        inputs.positionRotation,
        setState.goalSetpoint.getAsDouble(),
        ArmConstants.positionTolerance);
  }

  // 0.4257815 - x =  0.174560921875 +

  @RequiredArgsConstructor
  public enum ArmState {
    IDLE(() -> 0.055634078),
    INTAKE(() -> 0),
    SCORE(() -> 0.145634078);

    public final DoubleSupplier goalSetpoint;
  }
}
