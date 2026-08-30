package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX m_flywheelLeaderMotor;
  private final TalonFX m_flywheelFollowerMotor;
  private final SmartMotorControllerConfig m_leaderMotorConfig;
  private final SmartMotorControllerConfig m_followerMotorConfig;
  private final SmartMotorController m_leaderMotor;
  private final SmartMotorController m_followerMotor;
  private final FlyWheel m_flywheel;

  public FlywheelSubsystem() {
    m_flywheelLeaderMotor = new TalonFX(FlywheelConstants.LEADER_CAN_ID);
    m_flywheelFollowerMotor = new TalonFX(FlywheelConstants.FOLLOWER_CAN_ID);
    m_leaderMotorConfig = FlywheelConstants.LEADER_SMC_CONFIG.withSubsystem(this);
    m_followerMotorConfig = FlywheelConstants.FOLLOWER_SMC_CONFIG.withSubsystem(this);
    m_followerMotor =
        new TalonFXWrapper(
            m_flywheelFollowerMotor, FlywheelConstants.FOLLOWER_MOTOR, m_followerMotorConfig);
    m_leaderMotor =
        new TalonFXWrapper(
            m_flywheelLeaderMotor,
            FlywheelConstants.LEADER_MOTOR,
            m_leaderMotorConfig.withLooselyCoupledFollowers(m_followerMotor));
    m_flywheel = new FlyWheel(FlywheelConstants.FLY_WHEEL_CONFIG, m_leaderMotor);
  }

  /**
   * Returns the current flywheel angular velocity as measured by the motor encoder.
   *
   * @return Current flywheel speed.
   */
  public AngularVelocity getVelocity() {
    return m_flywheel.getSpeed();
  }

  /**
   * Returns the current flywheel velocity setpoint, if one is active. If no setpoint is active,
   * returns zero.
   *
   * @return Current flywheel velocity setpoint.
   */
  private AngularVelocity getVelocitySetpoint() {
    return m_flywheel.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  /**
   * Runs the flywheel at a fixed target velocity. The closed-loop controller and feedforward
   * maintain this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the flywheel (after gearing).
   * @return A command that holds the given speed while scheduled.
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_flywheel.runTo(speed, FlywheelConstants.TOLERANCE);
  }

  /**
   * Drives the flywheel in open-loop at a fixed duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_flywheel.set(dutyCycle);
  }

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven speed control
   * where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that continuously polls the supplier and updates the setpoint.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_flywheel.runTo(speed, FlywheelConstants.TOLERANCE);
  }

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for open-loop use
   * cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_flywheel.set(dutyCycle);
  }

  /**
   * Stops the flywheel by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_leaderMotor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public boolean isAtSetpoint() {
    return m_flywheel
        .getMechanismSetpointVelocity()
        .map(setpoint -> m_flywheel.isNear(setpoint, FlywheelConstants.TOLERANCE).getAsBoolean())
        .orElse(false);
  }

  @Override
  public void periodic() {
    m_flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheel.simIterate();

    SmartDashboard.putNumber("ShooterFlywheelMech/setpoint (RPM)", getVelocitySetpoint().in(RPM));
    SmartDashboard.putNumber("ShooterFlywheelMech/velocity (RPM)", getVelocity().in(RPM));
  }
}
