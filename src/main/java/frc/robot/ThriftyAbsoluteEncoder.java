
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Absolute encoder wrapper for Thriftybot using WPILib DutyCycleEncoder. */
public final class ThriftyAbsoluteEncoder {
  private final DutyCycleEncoder encoder;
  private final double absoluteOffset;

  private double lastAbsolute;
  private boolean hasLastValue = false;
  private double relativePosition;

  /** Creates the encoder using only a PWM port. */
  public ThriftyAbsoluteEncoder(int pwmPort) {
    this(pwmPort, 0.0);
    
  }

  /** Creates the encoder using a PWM port and a full-turn absolute offset (rotations). */
  public ThriftyAbsoluteEncoder(int pwmPort, double absoluteOffset) {
    this.encoder = new DutyCycleEncoder(pwmPort);
    this.absoluteOffset = absoluteOffset;
  }

  /** Returns absolute position in rotations [0, 1). */
  public double getAbsoluteRotations() {
    return wrapToUnit(encoder.get() - absoluteOffset);
  }

  public double getAbsoluteDegrees() {
    return getAbsoluteRotations() * 360.0;
  }

  public double getAbsoluteRadians() {
    return Math.toRadians(getAbsoluteDegrees());
  }

  /** Returns relative position in rotations, unwrapped across wrap-around. */
  public double getRelativeRotations() {
    final double absolute = getAbsoluteRotations();
    if (!hasLastValue) {
      lastAbsolute = absolute;
      hasLastValue = true;
      relativePosition = 0.0;
      return 0.0;
    }

    double delta = absolute - lastAbsolute;
    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    relativePosition += delta;
    lastAbsolute = absolute;
    return relativePosition;
  }

  public double getRelativeDegrees() {
    return getRelativeRotations() * 360.0;
  }

  public double getRelativeRadians() {
    return getRelativeRotations() * (2.0 * Math.PI);
  }

  /** Sets the relative position in rotations without changing the underlying absolute source. */
  public void setRelativePosition(double rotations) {
    final double absolute = getAbsoluteRotations();
    relativePosition = rotations;
    lastAbsolute = absolute;
    hasLastValue = true;
  }

  private static double wrapToUnit(double value) {
    double wrapped = value - Math.floor(value);
    if (wrapped < 0.0) {
      wrapped += 1.0;
    }
    return wrapped;
  }
}