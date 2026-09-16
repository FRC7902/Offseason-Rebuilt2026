package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class LaunchConstants {
  public static final double xPassTarget = Units.inchesToMeters(37);
  public static final double yPassTarget = Units.inchesToMeters(65);

  // TODO: Fill in robotToLauncher
  public static final Transform3d robotToLauncher = new Transform3d();

  // TODO: Requires actual values
  public static final double minDist = 0.1;
  public static final double maxDist = 5;
  public static final double phaseDelay = 0.03;

  // Hub shooting maps
  public static final InterpolatingTreeMap<Double, Angle> hoodAngleMap =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (start, end, t) -> start.plus(end.minus(start).times(t)));
  public static final InterpolatingTreeMap<Double, AngularVelocity> flywheelSpeedMap =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (start, end, t) -> start.plus(end.minus(start).times(t)));
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  // Passing Maps
  public static final InterpolatingTreeMap<Double, Angle> passingHoodAngleMap =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (start, end, t) -> start.plus(end.minus(start).times(t)));
  public static final InterpolatingTreeMap<Double, AngularVelocity> passingFlywheelSpeedMap =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (start, end, t) -> start.plus(end.minus(start).times(t)));
  public static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Values
  static {
    // Shooting
    hoodAngleMap.put(2.0091656738548784, Degrees.of(0.0));
    hoodAngleMap.put(2.267780124935476, Degrees.of(0.0));
    hoodAngleMap.put(2.5035564845846334, Degrees.of(4));
    hoodAngleMap.put(3.044637488210458, Degrees.of(5));
    hoodAngleMap.put(3.5518793699088248, Degrees.of(5));
    hoodAngleMap.put(5.058810145612536, Degrees.of(10));
    hoodAngleMap.put(7.0124000271948253, Degrees.of(13));

    flywheelSpeedMap.put(1.503106488548527, RPM.of(1700));
    flywheelSpeedMap.put(2.0091656738548784, RPM.of(1900));
    flywheelSpeedMap.put(2.267780124935476, RPM.of(2000));
    flywheelSpeedMap.put(2.5035564845846334, RPM.of(2056));
    flywheelSpeedMap.put(3.044637488210458, RPM.of(2100));
    flywheelSpeedMap.put(3.5518793699088248, RPM.of(2350));
    flywheelSpeedMap.put(5.058810145612536, RPM.of(2600));
    flywheelSpeedMap.put(7.0124000271948253, RPM.of(3000));

    // TODO: Update TOF based on `Thing` thread in #programming
    timeOfFlightMap.put(1.63, 1.017);
    timeOfFlightMap.put(2.40, 0.967);
    timeOfFlightMap.put(3.25, 1.19);
    timeOfFlightMap.put(4.15, 1.18);
    timeOfFlightMap.put(4.875, 1.25);

    // Passing TODO: Update Values
    passingHoodAngleMap.put(4.8533, Degrees.of(40));
    passingHoodAngleMap.put(16.00, Degrees.of(40));

    passingFlywheelSpeedMap.put(4.8533, RPM.of(3500));
    passingFlywheelSpeedMap.put(7.0573, RPM.of(4600));
    passingFlywheelSpeedMap.put(10.2642, RPM.of(5800));
    passingFlywheelSpeedMap.put(11.8006, RPM.of(6300d));
    passingFlywheelSpeedMap.put(13d, RPM.of(6300));
    passingFlywheelSpeedMap.put(16d, RPM.of(6300));

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);
  }
}
