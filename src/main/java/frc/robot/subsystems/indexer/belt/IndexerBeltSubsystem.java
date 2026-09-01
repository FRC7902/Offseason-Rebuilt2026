package frc.robot.subsystems.indexer.belt;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IndexerBeltSubsystem extends SubsystemBase {
  private final TalonFX m_indexerBeltLeaderMotor;
  private final TalonFX m_indexerBeltFollowerMotor;
  private final SmartMotorControllerConfig m_leaderMotorConfig;
  private final SmartMotorControllerConfig m_followerMotorConfig;
  private final SmartMotorController m_leaderMotor;
  private final SmartMotorController m_followerMotor;
  private final FlyWheel m_indexerBelt;

  public IndexerBeltSubsystem() {
    m_indexerBeltLeaderMotor = new TalonFX(IndexerBeltConstants.LEADER_CAN_ID);
    m_indexerBeltFollowerMotor = new TalonFX(IndexerBeltConstants.FOLLOWER_CAN_ID);
    m_leaderMotorConfig = IndexerBeltConstants.LEADER_SMC_CONFIG.withSubsystem(this);
    m_followerMotorConfig = IndexerBeltConstants.FOLLOWER_SMC_CONFIG.withSubsystem(this);
    m_followerMotor =
        new TalonFXWrapper(
            m_indexerBeltFollowerMotor, IndexerBeltConstants.FOLLOWER_MOTOR, m_followerMotorConfig);
    m_leaderMotor =
        new TalonFXWrapper(
            m_indexerBeltLeaderMotor,
            IndexerBeltConstants.LEADER_MOTOR,
            m_leaderMotorConfig.withLooselyCoupledFollowers(m_followerMotor));
    m_indexerBelt = new FlyWheel(IndexerBeltConstants.FLY_WHEEL_CONFIG, m_leaderMotor);
  }

  /**
   * Returns the current indexer belt angular velocity as measured by the motor encoder.
   *
   * @return Current indexer belt speed.
   */
  public AngularVelocity getVelocity() {
    return m_indexerBelt.getSpeed();
  }

  /**
   * Returns the current indexer belt velocity setpoint, if one is active. If no setpoint is active,
   * returns zero.
   *
   * @return Current indexer belt velocity setpoint.
   */
  private AngularVelocity getVelocitySetpoint() {
    return m_indexerBelt.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  /**
   * Runs the indexer belt at a fixed target velocity. The closed-loop controller and feedforward
   * maintain this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the indexer belt (after gearing).
   * @return A command that holds the given speed while scheduled.
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_indexerBelt.runTo(speed, IndexerBeltConstants.TOLERANCE);
  }

  /**
   * Drives the indexer belt in open-loop at a fixed duty cycle.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {
    return m_indexerBelt.set(dutyCycle);
  }

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven speed control
   * where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that continuously polls the supplier and updates the setpoint.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_indexerBelt.runTo(speed, IndexerBeltConstants.TOLERANCE);
  }

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for open-loop use
   * cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_indexerBelt.set(dutyCycle);
  }

  /**
   * Stops the indexer belt by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_leaderMotor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  /**
   * Runs a SysId routine on the indexer belt mechanism. This command will run a series of
   * quasistatic and dynamic tests, logging the results to the Phoenix SignalLogger. The routine
   * will stop the closed-loop controller before starting and restart it after finishing. The
   * routine will also log the state of the test to the SignalLogger.
   *
   * @return A command that runs the SysId routine.
   */
  public Command sysId() {
    final VoltageOut m_voltReq = new VoltageOut(0.0);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                // The voltage ramp rate used for quasistatic test routines. Defaults to 1 volt
                // per second if left null.
                null,
                // The step voltage output used for dynamic test routines. Defaults to 7 volts
                // if left null.
                null,
                // Safety timeout for the test routine commands. Defaults to 10 seconds if
                // left null.
                null,
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    m_indexerBeltLeaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));

    Command group =
        Commands.print("Starting SysId!")
            .beforeStarting(Commands.runOnce(m_leaderMotor::stopClosedLoopController))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
            .finallyDo(m_leaderMotor::startClosedLoopController)
            .andThen(Commands.print(getName() + " SysId test done."));

    return group.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
  }

  @Override
  public void periodic() {
    m_indexerBelt.updateTelemetry();

    SmartDashboard.putNumber("IndexerBeltMech/setpoint (RPM)", getVelocitySetpoint().in(RPM));
    SmartDashboard.putNumber("IndexerBeltMech/velocity (RPM)", getVelocity().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    m_indexerBelt.simIterate();
  }
}
