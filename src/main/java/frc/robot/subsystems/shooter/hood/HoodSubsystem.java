package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX m_hoodMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final Arm m_hood;

  public HoodSubsystem() {
    m_hoodMotor = new TalonFX(HoodConstants.CAN_ID);
    m_motorConfig = HoodConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_hoodMotor, HoodConstants.MOTOR, m_motorConfig);
    m_hood = new Arm(HoodConstants.ARM_CONFIG, m_motor);
  }

  /**
   * Drives the hood open-loop at the given duty cycle. Useful for manual adjustment and as a safe
   * default command.
   *
   * @param dutyCycle Output fraction in [-1, 1]; positive moves the hood in the positive direction.
   * @return Command that runs while scheduled and stops when interrupted.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_hood.set(dutyCycle);
  }

  /**
   * Moves the hood to a fixed angle using the closed-loop controller. The trapezoidal profile ramps
   * velocity so the hood does not slam into the setpoint.
   *
   * @param angle Target angle. Must be within the configured soft limits.
   * @return Command that runs until the hood reaches the target angle within tolerance.
   */
  public Command setAngle(Angle angle) {
    return m_hood.runTo(angle, HoodConstants.TOLERANCE);
  }

  /**
   * Stops the hood by disabling closed-loop control and commanding zero duty cycle.
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
    return m_hood
        .getMechanismSetpoint()
        .map(setpoint -> m_hood.isNear(setpoint, HoodConstants.TOLERANCE).getAsBoolean())
        .orElse(false);
  }

  @Override
  public void periodic() {
    m_hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_hood.simIterate();
  }
}
