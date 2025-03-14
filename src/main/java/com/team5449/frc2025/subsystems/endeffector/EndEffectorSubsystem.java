// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {
  private final DigitalInput digitalInput;
  private static final double intakeLatency = 0.07;
  private static final double outtakeLatency = 0.07;

  // private final ColorSensorV3 colorSensor;
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffectorSubsystem(EndEffectorIO io) {
    // colorSensor = new ColorSensorV3(Port.kOnboard);
    digitalInput = new DigitalInput(0);
    this.io = io;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return digitalInput.get();
  }

  public Command intake() {
    return runEnd(() -> io.setOpenLoop(0.7), () -> io.setOpenLoop(0))
        .until(
            /*new Trigger(() -> inputs.accelerationRPM < -500)*/ new Trigger(this::hasCoral)
                .debounce(intakeLatency));
  }

  public Command reverse() {
    return runEnd(() -> io.setOpenLoop(-0.5), () -> io.setOpenLoop(0));
  }

  public Command outtake() {
    return runEnd(() -> io.setOpenLoop(1), () -> io.setOpenLoop(0));
  }

  public Command outtakeAuto() {
    return runEnd(() -> io.setOpenLoop(1), () -> io.setOpenLoop(0))
        .until(new Trigger(() -> !hasCoral()).debounce(outtakeLatency));
  }

  public Command l1Outtake() {
    return runEnd(() -> io.differentialOpenLoop(0.5, 0.18), () -> io.differentialOpenLoop(0, 0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
