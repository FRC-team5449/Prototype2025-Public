// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorInputs {
    public boolean motorConnected = false;
    public AngularVelocity velocity = AngularVelocity.ofBaseUnits(0, RPM);
    public Angle position = Angle.ofBaseUnits(0, Rotation);
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  default void updateInputs(MotorInputs inputs) {}

  default void setOpenLoopDutyCycle(double dutyCycle) {}

  // These are in the "units" of the subsystem (rad, m).
  default void setPositionSetpoint(Angle position) {}

  default void setMotionMagicSetpoint(Angle position) {}

  default void setNeutralMode(NeutralModeValue mode) {}

  default void setVelocitySetpoint(AngularVelocity velocity) {}

  default void setCurrentPositionAsZero() {}

  default void setCurrentPosition(Angle position) {}

  default void setEnableSoftLimits(boolean forward, boolean reverse) {}
}
