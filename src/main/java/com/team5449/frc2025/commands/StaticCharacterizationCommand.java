// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.commands;

import com.team5449.lib.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class StaticCharacterizationCommand extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public StaticCharacterizationCommand(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput + " amps");
    SmartDashboard.putNumber("Reuslt", currentInput);
    inputConsumer.accept(0);
  }

  // 2.12

  // 3.706
}
