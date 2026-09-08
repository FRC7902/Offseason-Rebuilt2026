package frc.robot.subsystems.indexer.vertical_roller;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class VerticalRollerSubsystem extends SubsystemBase {
  private final SparkMax m_verticalRollerMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_verticalRoller;

  public VerticalRollerSubsystem() {
    m_verticalRollerMotor = new SparkMax(VerticalRollerConstants.CAN_ID, MotorType.kBrushless);
    m_motorConfig = VerticalRollerConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new SparkWrapper(m_verticalRollerMotor, VerticalRollerConstants.MOTOR, m_motorConfig);
    m_verticalRoller = new FlyWheel(VerticalRollerConstants.FLY_WHEEL_CONFIG, m_motor);
  }

  /** Returns the current vertical roller angular velocity as measured by the motor encoder. */
  public AngularVelocity getVelocity() {
    return m_verticalRoller.getSpeed();
  }

  /**
   * Returns the current vertical roller velocity setpoint, if one is active. If no setpoint is
   * active, returns zero.
   *
   * @return Current vertical roller velocity setpoint.
   */
  private AngularVelocity getVelocitySetpoint() {
    return m_verticalRoller.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  /**
   * Runs the vertical roller at a fixed target velocity. The closed-loop controller and feedforward
   * maintain this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the vertical roller (after gearing).
   * @return A command that runs until the vertical roller reaches the target speed within
   *     tolerance.
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_verticalRoller.runTo(speed, VerticalRollerConstants.TOLERANCE);
  }

  /**
   * Drives the vertical roller in open-loop at a fixed duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_verticalRoller.set(dutyCycle);
  }

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven speed control
   * where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that runs until the vertical roller reaches the target speed setpoint. The
   *     setpoint is continuously polled from the supplier, allowing for dynamic speed changes.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_verticalRoller.runTo(speed, VerticalRollerConstants.TOLERANCE);
  }

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for open-loop use
   * cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_verticalRoller.set(dutyCycle);
  }

  /**
   * Stops the vertical roller by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  @Override
  public void periodic() {
    m_verticalRoller.updateTelemetry();

    SmartDashboard.putNumber("VerticalRollerMech/setpoint (RPM)", getVelocitySetpoint().in(RPM));
    SmartDashboard.putNumber("VerticalRollerMech/velocity (RPM)", getVelocity().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    m_verticalRoller.simIterate();
  }
}
