// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ServoMotorSubsystemConfig {
  public String name = "UNNAMED";
  public int canMasterId;
  public boolean enableSlave = false;
  public boolean isSlaveOpposite = false;
  public int canSlaveId = -1;
  public String canBus;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();
  public TalonFXConfiguration fxSlaveConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon. rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) for sim
  public double momentOfInertia = 0.5;
}
