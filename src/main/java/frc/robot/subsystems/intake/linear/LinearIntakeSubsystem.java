package frc.robot.subsystems.intake.linear;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class LinearIntakeSubsystem extends SubsystemBase {
  private final TalonFX m_linearIntakeMotor;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController m_motor;
  ;
  private final Elevator m_linearIntake;

  public LinearIntakeSubsystem() {
    m_linearIntakeMotor = new TalonFX(LinearIntakeConstants.CAN_ID);
    motorConfig = LinearIntakeConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_linearIntakeMotor, LinearIntakeConstants.MOTOR, motorConfig);
    m_linearIntake = new Elevator(LinearIntakeConstants.ELEVATOR_CONFIG, m_motor);
  }

  /**
   * Open-loop duty-cycle command for manual control or testing. Positive duty cycle raises the
   * carriage; negative lowers it.
   *
   * @param dutyCycle Fraction of bus voltage, [-1.0, 1.0].
   * @return Command that runs until interrupted.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_linearIntake.set(dutyCycle);
  }

  /**
   * Closed-loop height command. Profiles to the target using the trapezoidal constraints defined in
   * motorConfig, then holds the carriage at that height via the PID controller.
   *
   * @param height Target carriage height in meters, clamped by soft limits [0, 2 m].
   * @return Command that runs until the carriage reaches the target height within the tolerance
   */
  public Command setHeight(Distance height) {
    return m_linearIntake.runTo(height, LinearIntakeConstants.TOLERANCE);
  }

  /**
   * Stops the linear intake by disabling closed-loop control and commanding zero duty cycle.
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
    m_linearIntake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_linearIntake.simIterate();
  }
}
