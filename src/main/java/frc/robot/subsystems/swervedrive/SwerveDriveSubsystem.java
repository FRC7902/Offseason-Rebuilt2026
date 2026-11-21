package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  private static SwerveDriveSubsystem m_instance;

  private static final String LIMELIGHT_A_NAME = "limelight-a";
  private static final String LIMELIGHT_B_NAME = "limelight-c";
  private boolean fastMode = false;

  private SwerveDriveSubsystem(File directory) {
    boolean blueAlliance = false;
    Pose2d startingPose =
        blueAlliance
            ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
            : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));

    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.POSE;
    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(SwerveDriveConstants.MAX_SPEED, startingPose);

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(
        false);

    swerveDrive.setCosineCompensator(
        false);

    swerveDrive.setAngularVelocityCompensation(
        true, true,
        0.1);

    swerveDrive.setModuleEncoderAutoSynchronize(
        false, 1);

    setupPathPlanner();
    setupLimelight();
  }

  public Command setSpeedCommand(boolean s) {
    return Commands.runOnce(() -> fastMode = s);
  }

  public boolean getSpeedMode() {
    return fastMode;
  }

  public static SwerveDriveSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }

    return m_instance;
  }

  private void setupPathPlanner() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;

      AutoBuilder.configure(
          this::getPose,

          this::resetOdometry,

          this::getRobotVelocity,

          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },

          new PPHolonomicDriveController(

              new PIDConstants(3.0, 0.0, 0.0),

              new PIDConstants(3.0, 0.0, 0.0)

              ),
          config,

          () -> {

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this

          );

    } catch (Exception e) {

      e.printStackTrace();
    }

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  private void setupLimelight() {
    LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHT_A_NAME);
    LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHT_B_NAME);
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public Command driveForward() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveBackward() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(-1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveLeft() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(0, 1), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveRight() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(0, -1), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {

          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3)
                  * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {

    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      swerveDrive.setGyroOffset(
          swerveDrive
              .getGyro()
              .getRawRotation3d()
              .minus(new Rotation3d(new Rotation2d(Degrees.of(180)))));

      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  private void localize() {
    addLimelightMeasurement(LIMELIGHT_A_NAME);
    addLimelightMeasurement(LIMELIGHT_B_NAME);
  }

  private void addLimelightMeasurement(String limelightName) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    double minTagAmbiguity =
        Arrays.stream(poseEstimate.rawFiducials)
            .mapToDouble(fiducial -> fiducial.ambiguity)
            .min()
            .orElse(Double.POSITIVE_INFINITY);

    if (poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 1 && minTagAmbiguity < 0.2) {
      double scalar = SwerveDriveConstants.BASE_STD_DEV * Math.pow(poseEstimate.avgTagDist, 2);
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(scalar, scalar, 9999999));
      swerveDrive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("fastMode", fastMode);
    localize();
  }

  public ChassisSpeeds getFieldSetpointVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public Rotation2d getRotation() {
    return swerveDrive.getPose().getRotation();
  }
}
