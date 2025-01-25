// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffector extends SubsystemBase {
  private final SparkMax endEffectorSpark;
  private final double currentThreshold = 10;
  private final double intakeLatency = 0.5;

  /** Creates a new EndEffector. */
  public EndEffector() {
    endEffectorSpark = new SparkMax(8, MotorType.kBrushless);
  }

  public void runOpenLoop(double speed) {
    endEffectorSpark.set(speed);
  }

  public Command intake() {
    return runEnd(() -> runOpenLoop(1), () -> runOpenLoop(0))
        .until(
            new Trigger(() -> endEffectorSpark.getOutputCurrent() > currentThreshold)
                .debounce(intakeLatency));
  }

  public Command outtake() {
    return runEnd(() -> runOpenLoop(1), () -> runOpenLoop(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
