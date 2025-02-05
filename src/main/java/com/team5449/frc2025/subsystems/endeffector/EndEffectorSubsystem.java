// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {
  private static final double currentThreshold = 10;
  private static final double intakeLatency = 0.5;

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffectorSubsystem(EndEffectorIO io) {
    this.io = io;
  }

  public Command intake() {
    return runEnd(() -> io.setOpenLoop(1), () -> io.setOpenLoop(0))
        .until(new Trigger(() -> inputs.currentAmps > currentThreshold).debounce(intakeLatency));
  }

  public Command outtake() {
    return runEnd(() -> io.setOpenLoop(1), () -> io.setOpenLoop(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
