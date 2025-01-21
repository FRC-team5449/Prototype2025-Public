// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.frc2025.commands;

import com.team5449.lib.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class FeedforwardCharacterizationCommand extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);

  private final List<Double> accelerationSamples = new LinkedList<>();
  private final List<Double> currentSamples = new LinkedList<>();

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier acceleratioSupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public FeedforwardCharacterizationCommand(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier acceleratioSupplier) {
    inputConsumer = characterizationInputConsumer;
    this.acceleratioSupplier = acceleratioSupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    accelerationSamples.clear();
    currentSamples.clear();
    timer.restart();
    inputConsumer.accept(0);
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
    currentSamples.add(currentInput);
    accelerationSamples.add(acceleratioSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    int n = accelerationSamples.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      sumX += accelerationSamples.get(i);
      sumY += currentSamples.get(i);
      sumXY += accelerationSamples.get(i) * currentSamples.get(i);
      sumX2 += accelerationSamples.get(i) * accelerationSamples.get(i);
    }
    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
    double kA = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

    NumberFormat formatter = new DecimalFormat("#0.00000");
    System.out.println("Static Characterization output: " + currentInput + " amps");
    System.out.println("\tkS: " + formatter.format(kS));
    System.out.println("\tkA: " + formatter.format(kA));
    inputConsumer.accept(0);
  }
}
