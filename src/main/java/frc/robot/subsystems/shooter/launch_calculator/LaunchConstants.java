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
    // Shooting TODO: Update Values
    hoodAngleMap.put(1.4625, Degrees.of(14));
    hoodAngleMap.put(2.1997, Degrees.of(20));
    hoodAngleMap.put(2.8742, Degrees.of(25));
    hoodAngleMap.put(3.4100, Degrees.of(30));
    hoodAngleMap.put(3.6500, Degrees.of(28));
    hoodAngleMap.put(4.0068, Degrees.of(25));
    hoodAngleMap.put(4.3896, Degrees.of(26));
    hoodAngleMap.put(4.7953, Degrees.of(30));

    flywheelSpeedMap.put(1.4625, RPM.of(3500));
    flywheelSpeedMap.put(2.1997, RPM.of(3600));
    flywheelSpeedMap.put(2.8742, RPM.of(3700));
    flywheelSpeedMap.put(3.4100, RPM.of(3900));
    flywheelSpeedMap.put(3.6500, RPM.of(3900));
    flywheelSpeedMap.put(4.0068, RPM.of(4325));
    flywheelSpeedMap.put(4.3896, RPM.of(4500));
    flywheelSpeedMap.put(4.7953, RPM.of(4550));

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
