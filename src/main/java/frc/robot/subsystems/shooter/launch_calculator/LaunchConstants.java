package frc.robot.subsystems.shooter.launch_calculator;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

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
  public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  // Passing Maps
  public static final InterpolatingDoubleTreeMap passingHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Values
  static {

    // hoodAngleMap.put(1.9176, Degrees.of(0.0));
    // flywheelSpeedMap.put(1.9176, RPM.of(1900));
    hoodAngleMap.put(2.0092, 0.0);
    flywheelSpeedMap.put(2.0092, 1900.0);
    timeOfFlightMap.put(1.9176, 0.935);

    // hoodAngleMap.put(3.5519, Degrees.of(5.0));
    // flywheelSpeedMap.put(3.5519, RPM.of(2350));
    hoodAngleMap.put(3.0446, 5.0);
    flywheelSpeedMap.put(3.0446, 2100.0);
    // timeOfFlightMap.put(3.5519, 1.947); TODO

    // hoodAngleMap.put(2.5036, Degrees.of(4.0));
    // flywheelSpeedMap.put(2.5036, RPM.of(2056));

    // hoodAngleMap.put(5.0588, Degrees.of(10.0));
    // flywheelSpeedMap.put(5.0588, RPM.of(2600));
    hoodAngleMap.put(2.2678, 0.0);
    flywheelSpeedMap.put(2.2678, 2000.0);
    timeOfFlightMap.put(5.0588, 1.204);

    // hoodAngleMap.put(7.0124, Degrees.of(13.0));
    // flywheelSpeedMap.put(7.0124, RPM.of(3000));
    timeOfFlightMap.put(7.0124, 1.502);

    // Passing TODO: Update Values
    passingHoodAngleMap.put(4.8533, 40.0);
    passingHoodAngleMap.put(16.00, 40.0);

    passingFlywheelSpeedMap.put(4.8533, 3500.0);
    passingFlywheelSpeedMap.put(7.0573, 4600.0);
    passingFlywheelSpeedMap.put(10.2642, 5800.0);
    passingFlywheelSpeedMap.put(11.8006, 6300.0);
    passingFlywheelSpeedMap.put(13.0, 6300.0);
    passingFlywheelSpeedMap.put(16.0, 6300.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);
  }
}
