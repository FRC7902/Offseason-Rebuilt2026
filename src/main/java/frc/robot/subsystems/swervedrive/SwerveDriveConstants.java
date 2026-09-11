package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public final class SwerveDriveConstants {
  public static final double MAX_SPEED =
      Units.feetToMeters(16); // TODO: Try increasing this to see if it
  // does
  // anything
  public static final double DRIVER_TRANSLATION_STICK_CURVE_EXPONENT = 2.0;

  public static final double AUTO_AIM_VELOCITY_COMPENSATION_FACTOR = 1.2; // TODO
  public static final Angle AUTO_AIM_ANGLE_TARGET_ERROR = Degrees.of(2.5);
  public static final Angle AUTO_AIM_SHOOTER_READY_FEEDING_ANGLE_TARGET_ERROR = Degrees.of(5);
  public static final Angle AUTO_AIM_SHOOTER_READY_ANGLE_TARGET_ERROR = Degrees.of(10);
  public static final double AUTO_AIM_SCALE_TRANSLATION = 0.15; // TODO

  // Extra distance past the starting line (towards the neutral zone) that would
  // still be considered "in the alliance zone" for the purposes of auto-aiming.
  public static final double ALLIANCE_ZONE_TOLERANCE_TO_STARTING_LINE = 0.1281386;

  public static final Translation2d BLUE_LEFT_FEEDING_TARGET =
      new Translation2d(0.567625 + 0.5, 6.05175);
  public static final Translation2d BLUE_RIGHT_FEEDING_TARGET =
      new Translation2d(0.567625 + 0.5, 2.01725);
  public static final Translation2d RED_LEFT_FEEDING_TARGET =
      new Translation2d(15.973375 - 0.5, 2.01725);
  public static final Translation2d RED_RIGHT_FEEDING_TARGET =
      new Translation2d(15.973375 - 0.5, 6.05175);
}
