package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
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
      // double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

  public LaunchingParameters getParameters() {

    if (latestParameters != null) {
      return latestParameters;
    }

    // TODO: Calculate launch parameters
    latestParameters =
        new LaunchingParameters(
            Rotation2d.fromDegrees(0), // driveAngle
            HoodConstants.MIN_ANGLE, // hoodAngle
            RPM.zero(), // flywheelSpeed
            TurretConstants.MIN_ANGLE, // turretAngle
            false); // passing

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
