package frc.robot.constants.enums;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotAltModes;
import frc.robot.utils.*;

public enum DrivetrainControl {
  COMP_CONTROL(
      KinematicLimits.fromDegrees(4.96824, 11.0, 450.0, 900.0),
      KinematicLimits.fromDegrees(4.96824, 3.0, 360.0, 360.0),
      15.0,
      Rotation2d.fromDegrees(900.0),
      new PIDFConstantsM(3.0, 0.0, 0.0, 0.0, Units.inchesToMeters(0.5)),
      PIDFConstantsRad.fromDegrees(2.0, 0.0, 0.0, 0.0, 1.0),
      PIDConstants.fromDegrees(1.85, 0.0, 0.0),
      PIDConstants.fromDegrees(1.75, 0.0, 0.0)
  ),
  PRACTICE_CONTROL(
      KinematicLimits.fromDegrees(4.96824, 15.0, 450.0, 900.0),
      KinematicLimits.fromDegrees(4.96824, 3.0, 360.0, 360.0),
      15.0,
      Rotation2d.fromDegrees(900.0),
      new PIDFConstantsM(3.0, 0.0, 0.0, 0.0, Units.inchesToMeters(0.5)),
      PIDFConstantsRad.fromDegrees(2.0, 0.0, 0.0, 0.0, 1.0),
      PIDConstants.fromDegrees(1.85, 0.0, 0.0),
      PIDConstants.fromDegrees(1.75, 0.0, 0.0)
  ),
  SWERVE_BASE_CONTROL(
      KinematicLimits.fromDegrees(4.96824, 15.0, 450.0, 900.0),
      KinematicLimits.fromDegrees(4.96824, 3.0, 360.0, 360.0),
      15.0,
      Rotation2d.fromDegrees(900.0),
      new PIDFConstantsM(10.0, 0.0, 0.2, 0.0, Units.inchesToMeters(2.0)),
      PIDFConstantsRad.fromDegrees(10.0, 0.0, 0.36, 0.0, 0.5),
      PIDConstants.fromDegrees(0.0, 0.0, 0.0),
      PIDConstants.fromDegrees(0.0, 0.0, 0.0)
  );

  public final KinematicLimits defaultLimits;
  public final KinematicLimits trapezoidalLimits;
  public final KinematicLimits slewingLimits;
  public final PIDFConstantsM xy;
  public final PIDFConstantsRad theta;
  public final PIDConstants autonTheta;
  public final PIDConstants chargerAutonTheta;

  DrivetrainControl(
      KinematicLimits defaultLimits,
      KinematicLimits trapezoidalLimits,
      KinematicLimits slewingLimits,
      PIDFConstantsM xy,
      PIDFConstantsRad theta,
      PIDConstants autonTheta,
      PIDConstants chargerAutonTheta
  ) {
    this.defaultLimits = defaultLimits;
    this.trapezoidalLimits = trapezoidalLimits;
    this.slewingLimits = slewingLimits;
    this.xy = xy;
    this.theta = theta;
    this.autonTheta = autonTheta;
    this.chargerAutonTheta = chargerAutonTheta;

    if (RobotAltModes.isTestMode) {
      this.defaultLimits.multiply(RobotAltModes.TEST_MODE_COEFFICIENT);
    }
  }

  DrivetrainControl(
      KinematicLimits defaultLimits,
      KinematicLimits trapezoidalLimits,
      double translationSlewingRate,
      Rotation2d angularSlewingRate,
      PIDFConstantsM xy,
      PIDFConstantsRad theta,
      PIDConstants autonTheta,
      PIDConstants chargerAutonTheta
  ) {
    this(
        defaultLimits,
        trapezoidalLimits,
        new KinematicLimits(
            defaultLimits.maxTranslationalVelocityMPS,
            translationSlewingRate,
            defaultLimits.maxAngularVelocityPS,
            angularSlewingRate
        ),
        xy,
        theta,
        autonTheta,
        chargerAutonTheta
    );
  }

  DrivetrainControl(
      KinematicLimits defaultLimits,
      KinematicLimits trapezoidalLimits,
      KinematicLimits slewingLimits,
      PIDFConstantsM xy,
      PIDFConstantsRad theta
  ) {
    this(
        defaultLimits,
        trapezoidalLimits,
        slewingLimits,
        xy,
        theta,
        PIDConstants.fromDegrees(1.85, 0.0, 0.0),
        PIDConstants.fromDegrees(1.75, 0.0, 0.0)
    );
  }

  DrivetrainControl(
      KinematicLimits defaultLimits,
      KinematicLimits trapezoidalLimits,
      double translationSlewingRate,
      Rotation2d angularSlewingRate,
      PIDFConstantsM xy,
      PIDFConstantsRad theta
  ) {
    this(
        defaultLimits,
        trapezoidalLimits,
        new KinematicLimits(
            defaultLimits.maxTranslationalVelocityMPS,
            translationSlewingRate,
            defaultLimits.maxAngularVelocityPS,
            angularSlewingRate
        ),
        xy,
        theta
    );
  }

  public ProfiledPIDController getAngularProfiledPIDController() {
    return theta.getProfiledController(trapezoidalLimits.getAngularTrapezoidalContraints());
  }

  public ProfiledPIDController getTranslationaProfiledPIDController() {
    return xy.getProfiledController(trapezoidalLimits.getTranslationalTrapezoidalContraints());
  }
}
