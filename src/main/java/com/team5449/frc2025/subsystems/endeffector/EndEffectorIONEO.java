// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorIONEO implements EndEffectorIO {
  private final SparkMax rollerLeft;
  private final SparkMax rollerRight;

  // Add variables to track previous velocity and timestamp
  private double prevVelocityRPM = 0.0;
  private double prevTimestamp = 0.0;

  public EndEffectorIONEO() {
    rollerLeft = new SparkMax(8, MotorType.kBrushless);
    rollerRight = new SparkMax(9, MotorType.kBrushless);
    rollerLeft.configure(
        new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    rollerRight.configure(
        new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.appliedVolts = rollerLeft.getBusVoltage();
    inputs.currentAmps = rollerLeft.getOutputCurrent();
    inputs.velocityRPM = rollerLeft.getEncoder().getVelocity();

    // Calculate acceleration based on velocity change over time
    double currentTime = System.currentTimeMillis() / 1000.0; // Convert to seconds
    double deltaTime = currentTime - prevTimestamp;

    if (deltaTime > 0) {
      inputs.accelerationRPM = (inputs.velocityRPM - prevVelocityRPM) / deltaTime;
    } else {
      inputs.accelerationRPM = 0.0;
    }

    // Update previous values for next calculation
    prevVelocityRPM = inputs.velocityRPM;
    prevTimestamp = currentTime;
  }

  @Override
  public void setOpenLoop(double outputVolts) {
    rollerLeft.set(outputVolts);
    rollerRight.set(outputVolts);
  }

  @Override
  public void differentialOpenLoop(double leftOutput, double rightOutput) {
    rollerLeft.set(leftOutput);
    rollerRight.set(rightOutput);
  }
}
