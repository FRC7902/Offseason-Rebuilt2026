package frc.robot.subsystems.indexer.roller;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RollerFloorSubsystem extends SubsystemBase {
  private final TalonFX m_rollerFloorMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_rollerFloor;

  public RollerFloorSubsystem() {
    m_rollerFloorMotor = new TalonFX(RollerFloorConstants.CAN_ID);
    m_motorConfig = RollerFloorConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_rollerFloorMotor, RollerFloorConstants.MOTOR, m_motorConfig);
    m_rollerFloor = new FlyWheel(RollerFloorConstants.FLY_WHEEL_CONFIG, m_motor);
  }

  /** Returns the current roller floor angular velocity as measured by the motor encoder. */
  public AngularVelocity getVelocity() {
    return m_rollerFloor.getSpeed();
  }

  /**
   * Runs the roller floor at a fixed target velocity. The closed-loop controller and feedforward
   * maintain this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the roller floor (after gearing).
   * @return A command that runs until the roller floor reaches the target speed within tolerance.
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_rollerFloor.runTo(speed, RollerFloorConstants.TOLERANCE);
  }

  /**
   * Drives the roller floor in open-loop at a fixed duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_rollerFloor.set(dutyCycle);
  }

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven speed control
   * where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that runs until the roller floor reaches the target speed setpoint. The
   *     setpoint is continuously polled from the supplier, allowing for dynamic speed changes.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_rollerFloor.runTo(speed, RollerFloorConstants.TOLERANCE);
  }

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for open-loop use
   * cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_rollerFloor.set(dutyCycle);
  }

  /**
   * Stops the roller floor by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(
        () -> {
          m_motor.stopClosedLoopController();
          setDutyCycle(0);
        });
  }

  @Override
  public void periodic() {
    m_rollerFloor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_rollerFloor.simIterate();
  }
}
