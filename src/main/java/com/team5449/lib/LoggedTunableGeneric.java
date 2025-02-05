// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5449.lib;

import com.team5449.frc2025.Constants;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableGeneric<T extends Measure<U>, U extends Unit> implements Supplier<T> {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final U defauleUnit;
  private T currentMeasure;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  @SuppressWarnings("unchecked")
  public LoggedTunableGeneric(String dashboardKey, U defaultUnit) {
    this.key = tableKey + "/" + dashboardKey;
    this.defauleUnit = defaultUnit;
    currentMeasure = (T) defaultUnit.of(defaultValue);
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableGeneric(String dashboardKey, double defaultValue, U defaultUnit) {
    this(dashboardKey, defaultUnit);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  @SuppressWarnings("unchecked")
  public double getDouble() {
    double value = 0.0;
    if (hasDefault) {
      value = Constants.tuningMode ? dashboardNumber.get() : defaultValue;
    }
    currentMeasure = (T) defauleUnit.of(value);
    return value;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = getDouble();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  @SafeVarargs
  public static <T extends Measure<U>, U extends Unit> void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableGeneric<T, U>... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(
          Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableGeneric::getDouble).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  @SafeVarargs
  public static <T extends Measure<U>, U extends Unit> void ifChanged(
      int id, Runnable action, LoggedTunableGeneric<T, U>... tunableNumbers) {
    ifChanged(id, (values) -> action.run(), tunableNumbers);
  }

  @Override
  public T get() {
    getDouble();
    return currentMeasure;
  }
}
