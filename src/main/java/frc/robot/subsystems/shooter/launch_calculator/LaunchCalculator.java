package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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
      // boolean isValid,
      Rotation2d driveAngle,
      // double driveVelocity,
      Angle hoodAngle,
      // double hoodVelocity,
      AngularVelocity flywheelSpeed,
      // double kickerSurfaceSpeed,
      // double distance,
      // double distanceNoLookahead,
       double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

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

    Pose2d estimatedPose = SwerveDriveSubsystem.getInstance().getPose();
    boolean alliance = SwerveDriveSubsystem.getAlliance() == DriverStation.Alliance.Red;
    boolean passing =
      (alliance) ?
        (estimatedPose.getX() > FieldConstants.RED_STARTING_LINE_X):(estimatedPose.getX()> FieldConstants.BLUE_STARTING_LINE_X);
    ChassisSpeeds robotRelativeVelocity = SwerveDriveSubsystem.getInstance().getRobotVelocity();
    estimatedPose =
      estimatedPose.exp(
        new Twist2d(
          robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
          robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
          robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
    Translation2d target =
      passing?
      getPassingTarget() : alliance? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    Pose2d launcherPosition = estimatedPose;//.transformBy(robotToLauncher.Transform2d());
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

    Pose2d lookaheadRobotPose =
      lookaheadPose.transformBy(toTransform2d(robotToLauncher));
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    Angle hoodAngle =
      (passing
              ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance)
              : hoodAngleMap.get(lookaheadLauncherToTargetDistance));
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle.in(Degrees))) lastHoodAngle = hoodAngle;

    AngularVelocity flywheelVelocity =
      passing
        ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
        : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    latestParameters =
      new LaunchingParameters(
        driveAngle,
        hoodAngle,
        flywheelVelocity,
        timeOfFlight,
        passing
      );
    return latestParameters;
  }
  public static Translation2d getPassingTarget(){
    return new Translation2d(xPassTarget, yPassTarget);
  }

  private static ChassisSpeeds transformVelocity(
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
  private static Rotation2d getDriveAngleWithLauncherOffset(
    Pose2d robotPose, Translation2d target) {
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
    boolean passing =
      LaunchCalculator.getInstance().getParameters().passing();

    Translation2d target =
      passing?
        getPassingTarget() : SwerveDriveSubsystem.getAlliance() == DriverStation.Alliance.Red ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;

    return new Pose2d(
      robotTranslation, getDriveAngleWithLauncherOffset(new Pose2d(robotTranslation, Rotation2d.kZero), target));
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
