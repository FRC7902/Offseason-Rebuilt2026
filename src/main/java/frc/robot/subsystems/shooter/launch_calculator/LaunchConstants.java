package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.hood.HoodConstants;

public class LaunchConstants {

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
    flywheelSpeedMap.put(1.9176, 1750.0);
    timeOfFlightMap.put(1.9176, 0.9350);

    hoodAngleMap.put(2.5036, 4.0);
    flywheelSpeedMap.put(2.5036, 1900.0);
    timeOfFlightMap.put(2.5035564845, 1.07);

    hoodAngleMap.put(3.5519, 5.0);
    flywheelSpeedMap.put(3.5519, 2150.0);
    timeOfFlightMap.put(3.5519, 1.16);

    hoodAngleMap.put(4.0, 7.0);
    flywheelSpeedMap.put(4.0, 2256.0);
    timeOfFlightMap.put(4.0, 1.185);

    hoodAngleMap.put(5.0588, 10.0);
    flywheelSpeedMap.put(5.0588, 2450.0);
    timeOfFlightMap.put(5.0588, 1.204);

    // TODO: re-tune these values for passing
    passingHoodAngleMap.put(4.8533, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(7.0573, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(10.2642, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(11.8006, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(13.0, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(14.0, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(15.0, HoodConstants.MAX_ANGLE.in(Degrees));
    passingHoodAngleMap.put(16.0, HoodConstants.MAX_ANGLE.in(Degrees));

    passingFlywheelSpeedMap.put(3.4350, 1800.0);
    passingFlywheelSpeedMap.put(4.8533, 2000.0);
    passingFlywheelSpeedMap.put(4.8533, 2000.0);
    passingFlywheelSpeedMap.put(7.0573, 2200.0);
    passingFlywheelSpeedMap.put(10.2642, 2400.0);
    passingFlywheelSpeedMap.put(11.8006, 2600.0);
    passingFlywheelSpeedMap.put(13.0, 2800.0);
    passingFlywheelSpeedMap.put(14.0, 3000.0);
    passingFlywheelSpeedMap.put(15.0, 3200.0);
    passingFlywheelSpeedMap.put(16.0, 3400.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);
  }
}
