package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchUtil.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class LaunchCalculator {
  private static LaunchCalculator m_instance;

  private LaunchCalculator() {}

  private Angle lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LaunchCalculator getInstance() {
    if (m_instance == null) {
      m_instance = new LaunchCalculator();
    }
    return m_instance;
  }

  public record LaunchingParameters(
      Angle turretAngle,
      Angle hoodAngle,
      AngularVelocity flywheelSpeed,
      double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDist);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDist);
  }

  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = SwerveDriveSubsystem.getInstance().getPose();
    boolean alliance = SwerveDriveSubsystem.getAlliance() == DriverStation.Alliance.Red;
    boolean passing =
        (alliance)
            ? (estimatedPose.getX() < FieldConstants.RED_STARTING_LINE_X)
            : (estimatedPose.getX() < FieldConstants.BLUE_STARTING_LINE_X);
    ChassisSpeeds robotRelativeVelocity = SwerveDriveSubsystem.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
    Translation2d target =
        passing
            ? getPassingTarget()
            : alliance ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    Pose2d launcherPosition = estimatedPose; // .transformBy(robotToLauncher.Transform2d());
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    var robotVelocity = SwerveDriveSubsystem.getInstance().getFieldSetpointVelocity();
    var robotAngle = SwerveDriveSubsystem.getInstance().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : transformVelocity(
                robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double effectiveTOF = timeOfFlight;
      double offsetX = launcherVelocity.vxMetersPerSecond * effectiveTOF;
      double offsetY = launcherVelocity.vyMetersPerSecond * effectiveTOF;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    Pose2d lookaheadRobotPose = lookaheadPose.transformBy(toTransform2d(robotToLauncher));
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    Angle turretAngle = getTurretAngleToHub(lookaheadRobotPose);

    Angle hoodAngle =
        (passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance)
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance));
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (lastHoodAngle == null || Double.isNaN(lastHoodAngle.in(Degrees))) lastHoodAngle = hoodAngle;

    AngularVelocity flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    latestParameters =
        new LaunchingParameters(turretAngle, hoodAngle, flywheelVelocity, timeOfFlight, passing);
    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
