package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class LaunchUtil {
  // Helpers
  public static Translation2d getPassingTarget() {
    return new Translation2d(xPassTarget, yPassTarget);
  }

  public static Angle getTurretAngleToHub(Pose2d robotPose) {
    Angle robotRotationCompensatedAngle =
        getAngleToAllianceHub(robotPose).minus(robotPose.getRotation().getMeasure());
    return wrapAngle(robotRotationCompensatedAngle);
  }

  private static Angle getAngleToAllianceHub(Pose2d robotPose) {
    Pose2d trueTurretPose = TurretSubsystem.getPose();
    Translation2d allianceHub = getAllianceHubTranslation2d();
    Translation2d hubDelta = allianceHub.minus(trueTurretPose.getTranslation());
    return hubDelta.getAngle().getMeasure();
  }

  private static Translation2d getAllianceHubTranslation2d() {
    DriverStation.Alliance alliance = null;
    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
    }
    if (alliance == DriverStation.Alliance.Red) {
      return Constants.RED_HUB_CENTER;
    }
    return Constants.BLUE_HUB_CENTER;
  }

  private static Angle wrapAngle(Angle angle) {
    Angle normalized = angle;

    if (normalized.lt(Degrees.of(0))) {
      normalized = normalized.plus(FULL_ROTATION);
    } else if (normalized.gte(FULL_ROTATION)) {
      normalized = normalized.minus(FULL_ROTATION);
    }

    if (normalized.lte(MAX_ANGLE)) {
      return normalized;
    } else if (normalized.gte(DEAD_ZONE)) {
      return normalized.minus(FULL_ROTATION);
    } else {
      Angle distanceToLow = normalized.minus(MAX_ANGLE);
      Angle distanceToHigh = DEAD_ZONE.minus(normalized);
      return distanceToLow.lt(distanceToHigh) ? MAX_ANGLE : MIN_ANGLE;
    }
  }

  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getSin()
                    + transform.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }

  public static Transform2d toTransform2d(Transform3d transform3d) {
    Translation3d t = transform3d.getTranslation();
    Rotation3d r = transform3d.getRotation();

    Translation2d translation2d = new Translation2d(t.getX(), t.getY());
    Rotation2d rotation2d = new Rotation2d(r.getZ()); // yaw only

    return new Transform2d(translation2d, rotation2d);
  }

  public static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    return fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
  }

  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    boolean passing = LaunchCalculator.getInstance().getParameters().passing();

    Translation2d target =
        passing
            ? getPassingTarget()
            : SwerveDriveSubsystem.getInstance().isRedAlliance()
                ? Constants.RED_HUB_CENTER
                : Constants.BLUE_HUB_CENTER;

    return new Pose2d(
        robotTranslation,
        getDriveAngleWithLauncherOffset(new Pose2d(robotTranslation, Rotation2d.kZero), target));
  }
}
