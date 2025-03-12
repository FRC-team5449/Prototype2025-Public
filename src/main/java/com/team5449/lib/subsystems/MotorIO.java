// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorInputs {
    public boolean motorConnected = false;
    public double velocityRotationPerSec = 0;
    public double positionRotation = 0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  public default void updateInputs(MotorInputs inputs) {}

  public default void setOpenLoopDutyCycle(double dutyCycle) {}

  // These are in the "units" of the subsystem (rad, m).
  public default void setPositionSetpoint(Angle position) {}

  public default void setMotionMagicSetpoint(Angle position) {}

  public default void setMotionMagicSetpoint(double positionRotation) {}

  public default void setNeutralMode(NeutralModeValue mode) {}

  public default void setVelocitySetpoint(AngularVelocity velocity) {}

  public default void setCurrentPositionAsZero() {}

  public default void setCurrentPosition(Angle position) {}

  public default void setEnableSoftLimits(boolean forward, boolean reverse) {}

  public default void setEnableSlaveMotor(boolean enableSlaveMotor) {}

  public default void runCharacterization(double currentAmps) {}
}
