package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem m_instance;

  private final TalonFX m_turretMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final CANcoder m_cancoder;
  private final Pivot m_turret;

  public Angle turretOffset;

  private TurretSubsystem() {
    m_turretMotor = new TalonFX(TurretConstants.CAN_ID);
    m_cancoder = new CANcoder(6);
    m_motorConfig =
        TurretConstants.SMC_CONFIG
            .withSubsystem(this)
            .withExternalEncoder(m_cancoder)
            .withExternalEncoderGearing(new MechanismGearing(GearBox.fromStages("3:5")))
            .withExternalEncoderZeroOffset(Degrees.of(164.3f * 3f / 5f));
    m_motor = new TalonFXWrapper(m_turretMotor, TurretConstants.MOTOR, m_motorConfig);
    m_turret = new Pivot(TurretConstants.PIVOT_CONFIG, m_motor);
    turretOffset = TurretConstants.DEFAULT_OFFSET_ANGLE;
  }

  public static TurretSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new TurretSubsystem();
    }
    return m_instance;
  }

  public static Pose2d getPose() {
    return getPose(SwerveDriveSubsystem.getInstance().getPose());
  }

  public static Pose2d getPose(Pose2d robotPose) {
    return robotPose.plus(
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
            TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()));
  }

  public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle) {
    Translation2d rRobot =
        TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d();
    Translation2d rWorld =
        rRobot.rotateBy(Rotation2d.fromRadians(robotAngle.in(Radians)));

    double omega = robotVelocity.omegaRadiansPerSecond;

    double vRotX = -omega * rWorld.getY();
    double vRotY = omega * rWorld.getX();

    double turretVx = robotVelocity.vxMetersPerSecond + vRotX;
    double turretVy = robotVelocity.vyMetersPerSecond + vRotY;

    double turretOmega = omega + m_motor.getMechanismVelocity().in(RadiansPerSecond);

    return new ChassisSpeeds(turretVx, turretVy, turretOmega);
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_turret.set(dutyCycle);
  }

  public Command setAngle(Angle angle) {
    return m_turret.runTo(angle, TurretConstants.TOLERANCE);
  }

  public Command setAngle(Supplier<Angle> angle) {
    return m_turret.runTo(angle, TurretConstants.TOLERANCE);
  }

  public void setAngleSetpoint(Angle measure) {
    m_turret.setMechanismPositionSetpoint(measure);
  }

  public Angle getAngle() {
    return m_turret.getAngle();
  }

  public Angle getAngleSetpoint() {
    return m_turret.getMechanismSetpoint().orElse(Degrees.of(0.0));
  }

  public Pose3d getPose3d() {

    return new Pose3d(
        new Translation3d(0.144, -0.152, 0.359),
        new Rotation3d(0.0, 0.0, getAngleSetpoint().in(Radians)));
  }

  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public Command sysId() {
    final VoltageOut m_voltReq = new VoltageOut(0.0);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(

                Volts.of(1).per(Second),

                Volts.of(1),

                Seconds.of(3),

                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> m_turretMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));

    Command group =
        Commands.print("Starting SysId!")
            .beforeStarting(Commands.runOnce(m_motor::stopClosedLoopController))
            .andThen(Commands.print("Running Quasistatic Forward."))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.print("Stopping Quasistatic Forward."))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.print("Running Quasistatic Reverse."))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.print("Stopping Quasistatic Reverse."))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.print("Running Dynamic Forward."))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.print("Stopping Dynamic Forward."))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.print("Running Dynamic Reverse."))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.print("Stopping Dynamic Reverse."))
            .finallyDo(m_motor::startClosedLoopController)
            .andThen(Commands.print(getName() + " SysId test done."));

    return group.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
  }

  public boolean isAtSetpoint() {
    return m_turret
        .getMechanismSetpoint()
        .map(setpoint -> m_turret.isNear(setpoint, TurretConstants.TOLERANCE).getAsBoolean())
        .orElse(false);
  }

  public Distance getDistanceToHub() {
    return Meters.of(
        getPose()
            .getTranslation()
            .getDistance(
                SwerveDriveSubsystem.getInstance().isRedAlliance()
                    ? FieldConstants.RED_HUB_CENTER
                    : FieldConstants.BLUE_HUB_CENTER));
  }

  public Distance getDistanceToHub(Pose2d turretPose) {
    return Meters.of(
        turretPose
            .getTranslation()
            .getDistance(
                SwerveDriveSubsystem.getInstance().isRedAlliance()
                    ? FieldConstants.RED_HUB_CENTER
                    : FieldConstants.BLUE_HUB_CENTER));
  }

  public Command moveTurretOffsetLeft() {
    return Commands.runOnce(() -> plusTurretOffset(2));
  }

  public Command moveTurretOffsetRight() {
    return Commands.runOnce(() -> plusTurretOffset(-2));
  }

  public Command resetTurretOffset() {
    return Commands.runOnce(() -> turretOffset = TurretConstants.DEFAULT_OFFSET_ANGLE);
  }

  public void plusTurretOffset(double measure) {
    turretOffset = turretOffset.plus(Degrees.of(measure));
  }

  @Override
  public void periodic() {
    m_turret.updateTelemetry();

    SmartDashboard.putNumber("TurretMech/setpoint (deg)", getAngleSetpoint().in(Degrees));
    SmartDashboard.putNumber("TurretMech/position (deg)", getAngle().in(Degrees));

    SmartDashboard.putBoolean("TurretMech/isAtSetpoint", isAtSetpoint());

    SmartDashboard.putNumber("TurretMech/turret-pose-x", getPose().getX());

    SmartDashboard.putNumber("TurretMech/turret-pose-y", getPose().getY());

    SmartDashboard.putNumber("TurretMech/turret-offset", turretOffset.in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    m_turret.simIterate();
  }
}
