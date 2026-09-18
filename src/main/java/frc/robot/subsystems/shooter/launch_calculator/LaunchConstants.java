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
    hoodAngleMap.put(1.9176, 0.0);
    flywheelSpeedMap.put(1.9176, 1900.0);
    timeOfFlightMap.put(1.9176, 0.9350);

    hoodAngleMap.put(2.5036, 4.0);
    flywheelSpeedMap.put(2.5036, 2056.0);
    timeOfFlightMap.put(2.5035564845, 1.07);

    hoodAngleMap.put(3.5519, 5.0);
    flywheelSpeedMap.put(3.5519, 2350.0);
    timeOfFlightMap.put(3.5519, 1.16);

    hoodAngleMap.put(5.0588, 10.0);
    flywheelSpeedMap.put(5.0588, 2600.0);
    timeOfFlightMap.put(5.0588, 1.204);

    hoodAngleMap.put(7.0124, 13.0);
    flywheelSpeedMap.put(7.0124, 3000.0);
    timeOfFlightMap.put(7.0124, 1.502);

    // TODO: re-tune these values for passing
    passingHoodAngleMap.put(4.8533, 43.3837);
    passingHoodAngleMap.put(7.0573, 43.3837);
    passingHoodAngleMap.put(10.2642, 43.3837);
    passingHoodAngleMap.put(11.8006, 43.3837);
    passingHoodAngleMap.put(13.0, 43.3837);
    passingHoodAngleMap.put(14.0, 43.3837);
    passingHoodAngleMap.put(15.0, 43.3837);
    passingHoodAngleMap.put(16.0, 43.3837);

    passingFlywheelSpeedMap.put(4.8533, 3500.0);
    passingFlywheelSpeedMap.put(7.0573, 4600.0);
    passingFlywheelSpeedMap.put(10.2642, 5800.0);
    passingFlywheelSpeedMap.put(11.8006, 6300.0);
    passingFlywheelSpeedMap.put(13.0, 6300.0);
    passingFlywheelSpeedMap.put(14.0, 6300.0);
    passingFlywheelSpeedMap.put(15.0, 6300.0);
    passingFlywheelSpeedMap.put(16.0, 6300.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);
  }
}
