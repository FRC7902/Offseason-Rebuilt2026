package frc.robot.subsystems.shooter.launch_calculator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchUtil.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class LaunchCalculator {
  private static LaunchCalculator m_instance;

  private LaunchCalculator() {}

  private Angle lastHoodAngle;
  private Rotation2d lastDriveAngle;

  private Pose2d lastLookaheadRobotPose = new Pose2d();

  private final StructPublisher<Pose2d> lookaheadRobotPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("/LaunchCalculator/LookaheadRobotPose", Pose2d.struct)
          .publish();

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
    boolean alliance = SwerveDriveSubsystem.getInstance().isRedAlliance();
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
    Pose2d launcherPosition = TurretSubsystem.getPose(estimatedPose);
    Distance launcherToTargetDistance = TurretSubsystem.getInstance().getDistanceToHub(launcherPosition);

    var robotVelocity = SwerveDriveSubsystem.getInstance().getFieldSetpointVelocity();
    var robotAngle = SwerveDriveSubsystem.getInstance().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : transformVelocity(
                robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance.in(Meters))
            : timeOfFlightMap.get(launcherToTargetDistance.in(Meters));
    Pose2d lookaheadPose = launcherPosition;
    Distance lookaheadLauncherToTargetDistance = launcherToTargetDistance;
    SmartDashboard.putNumber("launcherToTargetDistance", launcherToTargetDistance.in(Meters));

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance.in(Meters))
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance.in(Meters));
      double effectiveTOF = timeOfFlight;
      double offsetX = launcherVelocity.vxMetersPerSecond * effectiveTOF;
      double offsetY = launcherVelocity.vyMetersPerSecond * effectiveTOF;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = TurretSubsystem.getInstance().getDistanceToHub(lookaheadPose);
    }

    Pose2d lookaheadRobotPose = lookaheadPose.transformBy(toTransform2d(robotToLauncher));
    lastLookaheadRobotPose = lookaheadRobotPose;
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    Angle turretAngle = getTurretAngleToHub(lookaheadRobotPose);

    // Angle hoodAngle =
    //     (passing
    //         ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance)
    //         : hoodAngleMap.get(lookaheadLauncherToTargetDistance));

    Angle hoodAngle = Degrees.of(hoodAngleMap.get(lookaheadLauncherToTargetDistance.in(Meters)));

    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (lastHoodAngle == null || Double.isNaN(lastHoodAngle.in(Degrees))) lastHoodAngle = hoodAngle;

    // AngularVelocity flywheelVelocity =
    //     passing
    //         ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
    //         : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    AngularVelocity flywheelVelocity =
        RPM.of(flywheelSpeedMap.get(lookaheadLauncherToTargetDistance.in(Meters)));

    latestParameters =
        new LaunchingParameters(turretAngle, hoodAngle, flywheelVelocity, timeOfFlight, passing);
    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public void publishLookaheadRobotPose() {
    lookaheadRobotPosePublisher.set(lastLookaheadRobotPose);
  }
}
