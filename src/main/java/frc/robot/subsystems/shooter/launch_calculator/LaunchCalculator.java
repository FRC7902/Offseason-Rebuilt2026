package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;

public class LaunchCalculator {
  private static LaunchCalculator m_instance;

  private LaunchCalculator() {}

  public static LaunchCalculator getInstance() {
    if (m_instance == null) {
      m_instance = new LaunchCalculator();
    }
    return m_instance;
  }

  public record LaunchingParameters(
      // boolean isValid,
      Rotation2d driveAngle,
      // double driveVelocity,
      Angle hoodAngle,
      // double hoodVelocity,
      AngularVelocity flywheelSpeed,
      Angle turretAngle,
      // double kickerSurfaceSpeed,
      // double distance,
      // double distanceNoLookahead,
       double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

  private static final double minDist;
  private static final double maxDist;
  private static final double passingMinDist;
  private static final double passingMaxDist;
  private static final double phaseDelay;

  // Hub shooting maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
          new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
          new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
          new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
          new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
          new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
          new InterpolatingDoubleTreeMap();

  // Values
  static{
    //TODO: Requires actual values
    minDist = 0.1;
    maxDist = 5;
    passingMinDist = 3;
    passingMaxDist = 10;
    phaseDelay = 0.03;

    // Shooting TODO: Update Values
    hoodAngleMap.put(1.4625, Rotation2d.fromDegrees(14));
    hoodAngleMap.put(2.1997, Rotation2d.fromDegrees(20));
    hoodAngleMap.put(2.8742, Rotation2d.fromDegrees(25));
    hoodAngleMap.put(3.4100, Rotation2d.fromDegrees(30));
    hoodAngleMap.put(3.6500, Rotation2d.fromDegrees(28));
    hoodAngleMap.put(4.0068, Rotation2d.fromDegrees(25));
    hoodAngleMap.put(4.3896, Rotation2d.fromDegrees(26));
    hoodAngleMap.put(4.7953, Rotation2d.fromDegrees(30));

    flywheelSpeedMap.put(1.4625, 3500d);
    flywheelSpeedMap.put(2.1997, 3600d);
    flywheelSpeedMap.put(2.8742, 3700d);
    flywheelSpeedMap.put(3.4100, 3900d);
    flywheelSpeedMap.put(3.6500, 3900d);
    flywheelSpeedMap.put(4.0068, 4325d);
    flywheelSpeedMap.put(4.3896, 4500d);
    flywheelSpeedMap.put(4.7953, 4550d);

    timeOfFlightMap.put(1.63, 1.017);
    timeOfFlightMap.put(2.40, 0.967);
    timeOfFlightMap.put(3.25, 1.19);
    timeOfFlightMap.put(4.15, 1.18);
    timeOfFlightMap.put(4.875, 1.25);

    //Passing TODO: Update Values
    passingHoodAngleMap.put(4.8533, Rotation2d.fromDegrees(40));
    passingHoodAngleMap.put(16.00, Rotation2d.fromDegrees(40));

    passingFlywheelSpeedMap.put(4.8533, 3500d);
    passingFlywheelSpeedMap.put(7.0573, 4600d);
    passingFlywheelSpeedMap.put(10.2642, 5800d);
    passingFlywheelSpeedMap.put(11.8006, 6300d);
    passingFlywheelSpeedMap.put(13d, 6300d);
    passingFlywheelSpeedMap.put(16d, 6300d);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);
  }

  public static double getMinTimeOfFlight(){
    return timeOfFlightMap.get(minDist);
  }
  public static double getMaxTimeOfFlight(){
    return timeOfFlightMap.get(maxDist);
  }

  public LaunchingParameters getParameters() {

    if (latestParameters != null) {
      return latestParameters;
    }


    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
