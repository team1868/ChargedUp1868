package com.team254.lib.geometry;

import static com.team254.lib.util.Util.kEpsilon;

import com.team254.lib.util.Util;
import java.text.DecimalFormat;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Rotation2d254 implements IRotation2d<Rotation2d254> {
  public static final Rotation2d254 kIdentity = new Rotation2d254();
  public static final Rotation2d254 kPi = new Rotation2d254(Math.PI, false);
  public static final Rotation2d254 kHalfPi = new Rotation2d254(Math.PI / 2.0, false);

  public static Rotation2d254 identity() {
    return kIdentity;
  }

  protected double cos_angle_ = Double.NaN;
  protected double sin_angle_ = Double.NaN;
  protected double radians_ = Double.NaN;

  protected Rotation2d254(double x, double y, double radians) {
    cos_angle_ = x;
    sin_angle_ = y;
    radians_ = radians;
  }

  public Rotation2d254() {
    this(1.0, 0.0, 0.0);
  }

  public Rotation2d254(double radians, boolean normalize) {
    if (normalize) {
      radians = WrapRadians(radians);
    }
    radians_ = radians;
  }

  public Rotation2d254(double x, double y, boolean normalize) {
    if (normalize) {
      // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
      // we might accumulate rounding errors.
      // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
      double magnitude = Math.hypot(x, y);
      if (magnitude > kEpsilon) {
        sin_angle_ = y / magnitude;
        cos_angle_ = x / magnitude;
      } else {
        sin_angle_ = 0.0;
        cos_angle_ = 1.0;
      }
    } else {
      cos_angle_ = x;
      sin_angle_ = y;
    }
  }

  public Rotation2d254(final Rotation2d254 other) {
    cos_angle_ = other.cos_angle_;
    sin_angle_ = other.sin_angle_;
    radians_ = other.radians_;
  }

  public Rotation2d254(final edu.wpi.first.math.geometry.Rotation2d other) {
    cos_angle_ = other.getCos();
    sin_angle_ = other.getSin();
    radians_ = other.getRadians();
  }

  public Rotation2d254(final Translation2d direction, boolean normalize) {
    this(direction.x(), direction.y(), normalize);
  }

  public static Rotation2d254 fromRadians(double angle_radians) {
    return new Rotation2d254(angle_radians, true);
  }

  public static Rotation2d254 fromDegrees(double angle_degrees) {
    return fromRadians(Math.toRadians(angle_degrees));
  }

  public double cos() {
    ensureTrigComputed();
    return cos_angle_;
  }

  public double sin() {
    ensureTrigComputed();
    return sin_angle_;
  }

  public double tan() {
    ensureTrigComputed();
    if (Math.abs(cos_angle_) < kEpsilon) {
      if (sin_angle_ >= 0.0) {
        return Double.POSITIVE_INFINITY;
      } else {
        return Double.NEGATIVE_INFINITY;
      }
    }
    return sin_angle_ / cos_angle_;
  }

  public double getRadians() {
    ensureRadiansComputed();
    return radians_;
  }

  /**
   * Based on Team 1323's method of the same name.
   *
   * @return Rotation2d representing the angle of the nearest axis to the angle in standard position
   */
  public Rotation2d254 nearestPole() {
    double pole_sin = 0.0;
    double pole_cos = 0.0;
    if (Math.abs(cos_angle_) > Math.abs(sin_angle_)) {
      pole_cos = Math.signum(cos_angle_);
      pole_sin = 0.0;
    } else {
      pole_cos = 0.0;
      pole_sin = Math.signum(sin_angle_);
    }
    return new Rotation2d254(pole_cos, pole_sin, false);
  }

  public double getDegrees() {
    return Math.toDegrees(getRadians());
  }

  public Rotation2d254 unaryMinus() {
    return new Rotation2d254(-radians_, true);
  }

  public Rotation2d254 minus(Rotation2d254 other) {
    return rotateBy(other.unaryMinus());
  }

  public Rotation2d254 times(double scalar) {
    return new Rotation2d254(radians_ * scalar, true);
  }

  /**
   * We can rotate this Rotation2d by adding together the effects of it and
   * another rotation.
   *
   * @param other The other rotation. See:
   *              https://en.wikipedia.org/wiki/Rotation_matrix
   * @return This rotation rotated by other.
   */
  public Rotation2d254 rotateBy(final Rotation2d254 other) {
    if (hasTrig() && other.hasTrig()) {
      return new Rotation2d254(
          cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
          cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_,
          true
      );
    } else {
      return fromRadians(getRadians() + other.getRadians());
    }
  }

  @Override
  public Rotation2d254 mirror() {
    return Rotation2d254.fromRadians(-radians_);
  }

  public Rotation2d254 normal() {
    if (hasTrig()) {
      return new Rotation2d254(-sin_angle_, cos_angle_, false);
    } else {
      return fromRadians(getRadians() - Math.PI / 2.0);
    }
  }

  /**
   * The inverse of a Rotation2d "undoes" the effect of this rotation.
   *
   * @return The inverse of this rotation.
   */
  public Rotation2d254 inverse() {
    if (hasTrig()) {
      return new Rotation2d254(cos_angle_, -sin_angle_, false);
    } else {
      return fromRadians(-getRadians());
    }
  }

  /**
   * Obtain a Rotation2d that points in the opposite direction from this rotation.
   * @return This rotation rotated by 180 degrees.
   */
  public Rotation2d254 flip() {
    if (hasTrig()) {
      return new Rotation2d254(-cos_angle_, -sin_angle_, false);
    } else {
      return fromRadians(getRadians() + Math.PI);
    }
  }

  public boolean isParallel(final Rotation2d254 other) {
    if (hasRadians() && other.hasRadians()) {
      return Util.epsilonEquals(radians_, other.radians_)
          || Util.epsilonEquals(radians_, WrapRadians(other.radians_ + Math.PI));
    } else if (hasTrig() && other.hasTrig()) {
      return Util.epsilonEquals(sin_angle_, other.sin_angle_)
          && Util.epsilonEquals(cos_angle_, other.cos_angle_);
    } else {
      // Use public, checked version.
      return Util.epsilonEquals(getRadians(), other.getRadians())
          || Util.epsilonEquals(radians_, WrapRadians(other.radians_ + Math.PI));
    }
  }

  public Translation2d toTranslation() {
    ensureTrigComputed();
    return new Translation2d(cos_angle_, sin_angle_);
  }

  protected double WrapRadians(double radians) {
    final double k2Pi = 2.0 * Math.PI;
    radians = radians % k2Pi;
    radians = (radians + k2Pi) % k2Pi;
    if (radians > Math.PI)
      radians -= k2Pi;
    return radians;
  }

  private synchronized boolean hasTrig() {
    return !Double.isNaN(sin_angle_) && !Double.isNaN(cos_angle_);
  }

  private synchronized boolean hasRadians() {
    return !Double.isNaN(radians_);
  }

  private synchronized void ensureTrigComputed() {
    if (!hasTrig()) {
      assert (hasRadians());
      sin_angle_ = Math.sin(radians_);
      cos_angle_ = Math.cos(radians_);
    }
  }

  private synchronized void ensureRadiansComputed() {
    if (!hasRadians()) {
      assert (hasTrig());
      radians_ = Math.atan2(sin_angle_, cos_angle_);
    }
  }

  @Override
  public Rotation2d254 interpolate(final Rotation2d254 other, double x) {
    if (x <= 0.0) {
      return new Rotation2d254(this);
    } else if (x >= 1.0) {
      return new Rotation2d254(other);
    }
    double angle_diff = inverse().rotateBy(other).getRadians();
    return this.rotateBy(Rotation2d254.fromRadians(angle_diff * x));
  }

  @Override
  public String toString() {
    return "(" + new DecimalFormat("#0.000").format(getDegrees()) + " deg)";
  }

  @Override
  public String toCSV() {
    return new DecimalFormat("#0.000").format(getDegrees());
  }

  @Override
  public double distance(final Rotation2d254 other) {
    return inverse().rotateBy(other).getRadians();
  }

  @Override
  public Rotation2d254 add(Rotation2d254 other) {
    return this.rotateBy(other);
  }

  @Override
  public boolean equals(final Object other) {
    if (!(other instanceof Rotation2d254)) {
      return false;
    }

    return distance((Rotation2d254) other) < Util.kEpsilon;
  }

  @Override
  public Rotation2d254 getRotation() {
    return this;
  }
}
