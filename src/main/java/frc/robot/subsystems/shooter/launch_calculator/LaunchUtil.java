package frc.robot.subsystems.shooter.launch_calculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.*;
import static frc.robot.subsystems.shooter.launch_calculator.LaunchConstants.robotToLauncher;

public class LaunchUtil {
  // Helpers
  public static Translation2d getPassingTarget(){
    return new Translation2d(xPassTarget, yPassTarget);
  }

  public static Pose2d getPose(Pose2d robotPose) {
    return robotPose.plus(
      new Transform2d(
        TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
        TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()));
  }

  public static Angle getTurretAngleToHub(Pose2d robotPose) {
    Angle robotRotationCompensatedAngle =
      getAngleToAllianceHub(robotPose).minus(robotPose.getRotation().getMeasure());
    return wrapAngle(robotRotationCompensatedAngle);
  }

  private static Angle getAngleToAllianceHub(Pose2d robotPose) {
    Pose2d trueTurretPose = getPose(robotPose);
    Translation2d allianceHub = getAllianceHubTranslation2d();
    Translation2d hubDelta = allianceHub.minus(trueTurretPose.getTranslation());
    return hubDelta.getAngle().getMeasure();
  }

  private static Translation2d getAllianceHubTranslation2d() {
    DriverStation.Alliance alliance = null;
    if (DriverStation.getAlliance().isPresent()){
      alliance = DriverStation.getAlliance().get();
    }
    if (alliance == DriverStation.Alliance.Red) {
      return FieldConstants.RED_HUB_CENTER;
    }
    return FieldConstants.BLUE_HUB_CENTER;
  }

  private static Angle wrapAngle(Angle angle) {
    double degrees = angle.baseUnitMagnitude();

    degrees = ((degrees + 180) % 360 + 360) % 360 - 180;

    return Angle.ofBaseUnits(degrees, Degrees);
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
  public static Rotation2d getDriveAngleWithLauncherOffset(
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
}
