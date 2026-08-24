package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  private SwerveDrive drive;

  private final PIDController m_choreoControllerX = new PIDController(0.1, 0.0, 0.0); // TODO
  private final PIDController m_choreoControllerY = new PIDController(0.1, 0.0, 0.0); // TODO
  private final PIDController m_choreoControllerHeading = new PIDController(1.0, 0.0, 0.0); // TODO

  public SwerveDriveSubsystem() {
    SmartDashboard.putData(this);
    var cfg =
        new SwerveDriveConfig()
            .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
            .withSubsystem(this)
            .withTelemetry(TelemetryVerbosity.HIGH);
    try {
      // TODO: Update swerve configuration files
      drive =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
              .createSwerveDrive(cfg);
    } catch (Exception e) {
      System.out.println("Error creating swerve drive");
      System.out.println(e);
      throw new RuntimeException(e);
    }

    m_choreoControllerHeading.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveInputStream getAngularVelocityStream(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot);
  }

  public Command drive(SwerveInputStream stream) {
    return drive.drive(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                stream.get(), new Rotation2d(drive.getGyroAngle())));
  }

  /**
   * Create a {@link Command} that runs a full SysId characterization routine (quasistatic and
   * dynamic, forward and reverse) on a single swerve module's drive motor. The module's azimuth is
   * held pointed straight ahead for the duration of the test so only the drive motor is
   * characterized.
   *
   * @param moduleName Name of the module to test, e.g. "frontleft", "frontright", "backleft", or
   *     "backright".
   * @return {@link Command} that runs the full SysId routine on the given module.
   */
  public Command sysIdModule(String moduleName) {

    SwerveModule module = drive.getModule(moduleName).orElseThrow();
    SmartMotorController driveMotor = module.getDriveMotorController();
    SmartMotorController azimuthMotor = module.getAzimuthMotorController();

    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                azimuthMotor::setVoltage,
                log ->
                    log.motor(moduleName + "-azimuth")
                        .voltage(azimuthMotor.getVoltage())
                        .angularPosition(azimuthMotor.getMechanismPosition())
                        .angularVelocity(azimuthMotor.getMechanismVelocity()),
                this,
                moduleName + "-azimuth"));

    return Commands.runOnce(() -> azimuthMotor.setPosition(Rotation2d.kZero.getMeasure()))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
        .andThen(Commands.waitSeconds(1))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
        .andThen(Commands.waitSeconds(1))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
        .andThen(Commands.waitSeconds(1))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
        .withName("SysId " + moduleName + " Azimuth");
  }

  public void driveFieldRelative(ChassisSpeeds velocity) {
    drive.setFieldRelativeChassisSpeeds(velocity);
  }

  public void driveRobotRelative(ChassisSpeeds velocity) {
    drive.setRobotRelativeChassisSpeeds(velocity);
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    drive.resetOdometry(initialHolonomicPose);
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + m_choreoControllerX.calculate(pose.getX(), sample.x),
            sample.vy + m_choreoControllerY.calculate(pose.getY(), sample.y),
            sample.omega
                + m_choreoControllerHeading.calculate(
                    pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    driveFieldRelative(speeds);
  }

  public Command stop() {
    return new InstantCommand(() -> driveRobotRelative(new ChassisSpeeds(0, 0, 0)));
  }

  /** Zero the gyro heading. Bind this to a button combo for field recovery. */
  public Command zeroGyro() {
    return runOnce(() -> drive.zeroGyro());
  }

  public void periodic() {
    drive.updateTelemetry();
  }

  public void simulationPeriodic() {
    drive.simIterate();
  }
}
