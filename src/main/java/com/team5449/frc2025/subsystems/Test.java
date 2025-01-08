// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Millimeter;

import com.team5449.lib.LoggedTunableNumber;
import com.team5449.lib.LoggedTunbaleGeneric;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Test extends SubsystemBase {
  private static final LoggedTunbaleGeneric<Distance, DistanceUnit> a =
      new LoggedTunbaleGeneric<>("Test", 0.0, Meter);

  private static final LoggedTunableNumber b = new LoggedTunableNumber("Test2", 0);

  /** Creates a new Test. */
  public Test() {}

  @Override
  public void periodic() {
    Logger.recordOutput("Output Degree", b.getAsDouble());
    Logger.recordOutput("Output Radian", a.get().in(Millimeter));
  }
}
