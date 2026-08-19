package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.turret.TurretConstants;

import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX m_flywheelMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_flywheel;

  public FlywheelSubsystem() {
    m_flywheelMotor = new TalonFX(FlywheelConstants.CAN_ID);
    m_motorConfig = FlywheelConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_flywheelMotor, FlywheelConstants.MOTOR, m_motorConfig);
    m_flywheel = new FlyWheel(FlywheelConstants.FLY_WHEEL_CONFIG, m_motor);
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
    return this.runOnce(
        () -> {
          m_motor.stopClosedLoopController();
          setDutyCycle(0);
        });
  }

  public boolean isAtSetpoint() {
    return m_flywheel.getMechanismSetpointVelocity()
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
  }
}
