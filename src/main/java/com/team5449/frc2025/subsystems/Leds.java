// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Leds {
  private static Leds instance;
  private final Spark ledSpark;
  private LedState currentState = LedState.NoCoral;
  @Setter private boolean autoScoring = false;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  private Leds() {
    ledSpark = new Spark(0);
  }

  public void periodic() {
    if (autoScoring) {
      ledSpark.set(LedState.AutoScoring.ledPWMPower);
    } else {
      ledSpark.set(currentState.ledPWMPower);
    }
  }

  @RequiredArgsConstructor
  public enum LedState {
    AutoScoring(-0.43),
    Intaking(-0.07),
    HasCoral(0.73),
    NoCoral(0.91);

    private final double ledPWMPower;
  }
}
