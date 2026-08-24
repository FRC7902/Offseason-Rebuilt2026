package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  private SwerveDrive drive;

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

    try {
      setupPathPlanner();
    } catch (IOException | ParseException e) {
      throw new RuntimeException(
          "PathPlanner setup failed -- check deploy/pathplanner/settings.json exists", e);
    }
  }

  private void setupPathPlanner() throws IOException, ParseException {
    AutoBuilder.configure(
        drive::getPose, // robot pose supplier
        drive::resetOdometry, // called if an auto defines a starting pose
        drive::getRobotRelativeSpeed, // ChassisSpeeds supplier -- MUST be robot-relative
        (speedsRobotRelative, moduleFeedForwards) ->
            drive.setRobotRelativeChassisSpeeds(speedsRobotRelative),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // translation PID
            new PIDConstants(5.0, 0.0, 0.0) // rotation PID
            ),
        RobotConfig.fromGUISettings(), // reads deploy/pathplanner/settings.json
        () -> {
          // Field origin is always the blue alliance wall -- flip paths when on red.
          var alliance = DriverStation.getAlliance();
          return alliance.filter(a -> a == DriverStation.Alliance.Red).isPresent();
        },
        this // subsystem requirement for the generated commands
        );
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

  public void periodic() {
    drive.updateTelemetry();
  }

  public void simulationPeriodic() {
    drive.simIterate();
  }
}
