package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class LaunchUtil {
  // Helpers
  public static Translation2d getPassingTarget() {
    double robotY = SwerveDriveSubsystem.getInstance().getPose().getTranslation().getY();
    if (robotY >= FieldConstants.FIELD_WIDTH / 2) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        return FieldConstants.PASSING_UP_RED;
      } else {
        return FieldConstants.PASSING_UP_BLUE;
      }
    } else {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        return FieldConstants.PASSING_DOWN_RED;
      } else {
        return FieldConstants.PASSING_DOWN_BLUE;
      }
    }
  }

  public static Angle getTurretAngleToHub(Pose2d robotPose) {
    Angle angleToAllianceHub = getAngleToAllianceHub(robotPose);
    // Turret's zero setpoint faces the back of the robot, i.e. the robot's heading + 180 deg.
    Angle turretZeroFieldAngle = robotPose.getRotation().getMeasure().plus(Degrees.of(180));
    Angle robotRotationCompensatedAngle =
        angleToAllianceHub.minus(turretZeroFieldAngle).plus(Degrees.of(-3.444457));
    Angle wrappedAngle =
        wrapAngle(robotRotationCompensatedAngle, TurretSubsystem.getInstance().getAngle());

    SmartDashboard.putNumber(
        "LaunchCalculator/angleToAllianceHub (deg)", angleToAllianceHub.in(Degrees));
    SmartDashboard.putNumber(
        "LaunchCalculator/turretZeroFieldAngle (deg)", turretZeroFieldAngle.in(Degrees));
    SmartDashboard.putNumber(
        "LaunchCalculator/robotRotationCompensatedAngle (deg)",
        robotRotationCompensatedAngle.in(Degrees));
    SmartDashboard.putNumber("LaunchCalculator/wrappedTurretAngle (deg)", wrappedAngle.in(Degrees));

    return wrappedAngle;
  }

  public static Angle getTurretAngleToPassingTarget(Pose2d robotPose) {
    Angle angleToPassingTarget = getAngleToPassingTarget(robotPose);
    // Turret's zero setpoint faces the back of the robot, i.e. the robot's heading + 180 deg.
    Angle turretZeroFieldAngle = robotPose.getRotation().getMeasure().plus(Degrees.of(180));
    Angle robotRotationCompensatedAngle =
        angleToPassingTarget.minus(turretZeroFieldAngle).plus(Degrees.of(-3.4444567));
    Angle wrappedAngle =
        wrapAngle(robotRotationCompensatedAngle, TurretSubsystem.getInstance().getAngle());

    return wrappedAngle;
  }

  private static Angle getAngleToPassingTarget(Pose2d robotPose) {
    Pose2d turretPose = TurretSubsystem.getPose(robotPose);
    Translation2d passingTarget = getPassingTarget();
    Translation2d hubDelta = passingTarget.minus(turretPose.getTranslation());
    return hubDelta.getAngle().getMeasure();
  }

  private static Angle getAngleToAllianceHub(Pose2d robotPose) {
    Pose2d turretPose = TurretSubsystem.getPose(robotPose);
    Translation2d allianceHub = getAllianceHubTranslation2d();
    Translation2d hubDelta = allianceHub.minus(turretPose.getTranslation());
    return hubDelta.getAngle().getMeasure();
  }

  private static Translation2d getAllianceHubTranslation2d() {
    DriverStation.Alliance alliance = null;
    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
    }
    if (alliance == DriverStation.Alliance.Red) {
      return FieldConstants.RED_HUB_CENTER;
    }
    return FieldConstants.BLUE_HUB_CENTER;
  }

  private static Angle abs(Angle angle) {
    return angle.lt(Degrees.of(0)) ? angle.unaryMinus() : angle;
  }

  private static Angle wrapAngle(Angle target, Angle currentAngle) {
    Angle normalized = target;

    while (normalized.gte(MAX_ANGLE)) {
      normalized = normalized.minus(FULL_ROTATION);
    }
    while (normalized.lt(MIN_ANGLE)) {
      normalized = normalized.plus(FULL_ROTATION);
    }
    Angle alternate = normalized.plus(FULL_ROTATION);
    if (alternate.lte(MAX_ANGLE)) {
      Angle distanceToNormalized = abs(normalized.minus(currentAngle));
      Angle distToAlternate = abs(alternate.minus(currentAngle));
      if (distToAlternate.lt(distanceToNormalized)) {
        return alternate;
      }
    }
    return normalized;
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
}
