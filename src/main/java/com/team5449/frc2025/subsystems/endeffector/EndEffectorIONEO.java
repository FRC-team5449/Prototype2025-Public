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
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorIONEO implements EndEffectorIO {
  private final SparkMax endEffector;

  public EndEffectorIONEO() {
    endEffector = new SparkMax(0, MotorType.kBrushless);
    endEffector.configure(
        new SparkMaxConfig().inverted(true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.appliedVolts = endEffector.getBusVoltage();
    inputs.currentAmps = endEffector.getOutputCurrent();
  }

  @Override
  public void setOpenLoop(double outputVolts) {
    endEffector.set(outputVolts);
  }
}
