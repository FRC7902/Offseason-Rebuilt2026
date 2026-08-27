package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX m_feederMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_feeder;

  public FeederSubsystem() {
    m_feederMotor = new TalonFX(FeederConstants.CAN_ID);
    m_motorConfig = FeederConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_feederMotor, FeederConstants.MOTOR, m_motorConfig);
    m_feeder = new FlyWheel(FeederConstants.FLY_WHEEL_CONFIG, m_motor);
  }

  /**
   * Returns the current feeder angular velocity as measured by the motor encoder.
   *
   * @return Current feeder speed.
   */
  public AngularVelocity getVelocity() {
    return m_feeder.getSpeed();
  }

  /**
   * Runs the feeder at a fixed target velocity. The closed-loop controller and feedforward maintain
   * this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the feeder (after gearing).
   * @return A command that runs until the feeder reaches the target speed within tolerance.
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_feeder.runTo(speed, FeederConstants.TOLERANCE);
  }

  /**
   * Drives the feeder in open-loop at a fixed duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_feeder.set(dutyCycle);
  }

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven speed control
   * where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that runs until the feeder reaches the target speed within tolerance. The
   *     setpoint is continuously polled from the supplier, allowing for dynamic speed changes.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_feeder.runTo(speed, FeederConstants.TOLERANCE);
  }

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for open-loop use
   * cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_feeder.set(dutyCycle);
  }

  /**
   * Stops the feeder by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return Commands.parallel(
        this.runOnce(() -> m_motor.stopClosedLoopController()), setDutyCycle(0));
  }

  @Override
  public void periodic() {
    m_feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feeder.simIterate();
  }
}
