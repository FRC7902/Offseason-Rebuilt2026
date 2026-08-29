package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX m_turretMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final Pivot m_turret;

  public TurretSubsystem() {
    m_turretMotor = new TalonFX(TurretConstants.CAN_ID);
    m_motorConfig = TurretConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_turretMotor, TurretConstants.MOTOR, m_motorConfig);
    m_turret = new Pivot(TurretConstants.PIVOT_CONFIG, m_motor);
  }

  /**
   * Returns the turret's estimated pose in the field frame based on the robot's pose and the fixed
   * turret offset from the robot center.
   *
   * @param robotPose Current pose of the robot in the field coordinate system.
   * @return Field-relative pose of the turret mounting point.
   */
  public Pose2d getPose(Pose2d robotPose) {
    return robotPose.plus(
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
            TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()));
  }

  /**
   * Computes the turret's linear and angular velocity in the field frame from the robot's chassis
   * motion and the turret's own motor rotation rate.
   *
   * @param robotVelocity Current chassis velocity in the robot frame.
   * @param robotAngle Current robot heading used to rotate the turret offset into the field frame.
   * @return Turret velocity in field coordinates.
   */
  public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle) {
    Translation2d rRobot =
        TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(); // in robot frame
    Translation2d rWorld =
        rRobot.rotateBy(Rotation2d.fromRadians(robotAngle.in(Radians))); // rotate into field
    // frame

    double omega = robotVelocity.omegaRadiansPerSecond; // robot yaw rate (rad/s)

    // rotational linear velocity at turret (v_rot = ω × r_world)
    double vRotX = -omega * rWorld.getY();
    double vRotY = omega * rWorld.getX();

    // final turret linear velocity in field frame
    double turretVx = robotVelocity.vxMetersPerSecond + vRotX;
    double turretVy = robotVelocity.vyMetersPerSecond + vRotY;

    // turret angular velocity in field frame
    double turretOmega = omega + m_motor.getMechanismVelocity().in(RadiansPerSecond);

    return new ChassisSpeeds(turretVx, turretVy, turretOmega);
  }

  /**
   * Drives the turret in open-loop at the given duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1]. Positive values move the turret in the positive
   *     direction.
   * @return Command that runs while scheduled and stops when interrupted.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_turret.set(dutyCycle);
  }

  /**
   * Moves the turret to a fixed angular setpoint using the closed-loop controller.
   *
   * @param angle Target turret angle.
   * @return Command that runs until the turret reaches the target angle within tolerance.
   */
  public Command setAngle(Angle angle) {
    return m_turret.runTo(angle, TurretConstants.TOLERANCE);
  }

  /**
   * Moves the turret to a variable angular setpoint using the closed-loop controller. The target
   * angle is continuously polled from the supplier, allowing for dynamic aiming.
   *
   * @param angle
   * @return Command that runs until the turret reaches the target angle within tolerance.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return m_turret.runTo(angle, TurretConstants.TOLERANCE);
  }

  /**
   * Sets the turret's mechanism position setpoint without creating a command.
   *
   * @param measure Desired turret angle reference.
   */
  public void setAngleSetpoint(Angle measure) {
    m_turret.setMechanismPositionSetpoint(measure);
  }

  public Angle getAngle() {
    return m_turret.getAngle();
  }

  public Pose3d getPose3d() {
    return new Pose3d(
        new Translation3d(0.144, -0.152, 0.359), new Rotation3d(0.0, 0.0, getAngle().in(Radians)));
  }

  /**
   * Stops the turret by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public boolean isAtSetpoint() {
    return m_turret
        .getMechanismSetpoint()
        .map(setpoint -> m_turret.isNear(setpoint, TurretConstants.TOLERANCE).getAsBoolean())
        .orElse(false);
  }

  @Override
  public void periodic() {
    m_turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_turret.simIterate();
  }
}
