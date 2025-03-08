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
import edu.wpi.first.wpilibj2.command.Command;

public class HopperSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public HopperSubsystem(final MotorIO io) {
    super(HopperConstants.kHopperConfig, new MotorInputsAutoLogged(), io);
    setCurrentPositionAsZero();
  }

  public Command decline() {
    return startEnd(() -> io.setOpenLoopDutyCycle(0.2), () -> io.setOpenLoopDutyCycle(0));
  }

  public Command elevate() {
    return startEnd(() -> io.setOpenLoopDutyCycle(-0.2), () -> io.setOpenLoopDutyCycle(0));
  }
}
