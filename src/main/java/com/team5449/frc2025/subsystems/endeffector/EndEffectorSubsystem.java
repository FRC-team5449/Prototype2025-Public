// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {
  private final DigitalInput coralSwitch;
  private static final double intakeLatency = 0;
  private static final double currentThreshold = 0.3;

  private final ColorSensorV3 colorSensor;
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffectorSubsystem(EndEffectorIO io) {
    coralSwitch = new DigitalInput(0);
    colorSensor = new ColorSensorV3(Port.kOnboard);
    this.io = io;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return colorSensor.getProximity() > 500;
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("R", colorSensor.getRed());
    SmartDashboard.putNumber("G", colorSensor.getGreen());
    SmartDashboard.putNumber("B", colorSensor.getBlue());
    SmartDashboard.putNumber("a", colorSensor.getProximity());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
