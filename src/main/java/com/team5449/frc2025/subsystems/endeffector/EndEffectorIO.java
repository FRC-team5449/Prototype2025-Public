// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double accelerationRPM = 0.0;
    public double velocityRPM = 0.0;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setOpenLoop(double outputVolts) {}

  public default void differentialOpenLoop(double leftOutput, double rightOutput) {}

  public default void setPosition(double position) {}
}
