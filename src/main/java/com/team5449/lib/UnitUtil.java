// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class UnitUtil {
  public static Angle clamp(Angle value, Angle low, Angle high) {
    return Angle.ofBaseUnits(
        MathUtil.clamp(value.in(Radian), low.in(Radian), high.in(Radian)), Radian);
  }
}
