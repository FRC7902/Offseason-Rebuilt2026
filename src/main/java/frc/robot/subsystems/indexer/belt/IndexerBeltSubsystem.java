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
  private static IndexerBeltSubsystem m_instance;

  private final TalonFX m_indexerBeltLeaderMotor;
  private final TalonFX m_indexerBeltFollowerMotor;
  private final SmartMotorControllerConfig m_leaderMotorConfig;
  private final SmartMotorControllerConfig m_followerMotorConfig;
  private final SmartMotorController m_leaderMotor;
  private final SmartMotorController m_followerMotor;
  private final FlyWheel m_indexerBelt;

  private IndexerBeltSubsystem() {
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

  public static IndexerBeltSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IndexerBeltSubsystem();
    }
    return m_instance;
  }

  public AngularVelocity getVelocity() {
    return m_indexerBelt.getSpeed();
  }

  private AngularVelocity getVelocitySetpoint() {
    return m_indexerBelt.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_indexerBelt.runTo(speed, IndexerBeltConstants.TOLERANCE);
  }

  public Command setDutyCycle(double dutyCycle) {

    return Commands.parallel(
            Commands.startRun(
                    m_leaderMotor::stopClosedLoopController,
                    () -> m_leaderMotor.setDutyCycle(dutyCycle))
                .finallyDo(m_leaderMotor::startClosedLoopController),
            Commands.startRun(
                    m_followerMotor::stopClosedLoopController,
                    () -> m_followerMotor.setDutyCycle(dutyCycle))
                .finallyDo(m_followerMotor::startClosedLoopController))
        .withName(this.getName() + " SetDutyCycle");
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_indexerBelt.runTo(speed, IndexerBeltConstants.TOLERANCE);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {

    return Commands.parallel(
            Commands.startRun(
                    m_leaderMotor::stopClosedLoopController,
                    () -> m_leaderMotor.setDutyCycle(dutyCycle.get()))
                .finallyDo(m_leaderMotor::startClosedLoopController),
            Commands.startRun(
                    m_followerMotor::stopClosedLoopController,
                    () -> m_followerMotor.setDutyCycle(dutyCycle.get()))
                .finallyDo(m_followerMotor::startClosedLoopController))
        .withName(this.getName() + " SetDutyCycle");
  }

  public Command stop() {
    return this.runOnce(() -> m_leaderMotor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public Command sysId() {
    final VoltageOut m_voltReq = new VoltageOut(0.0);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(

                null,

                null,

                null,

                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    m_indexerBeltLeaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));

    Command group =
        Commands.print("Starting SysId!")
            .beforeStarting(Commands.runOnce(m_leaderMotor::stopClosedLoopController))
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
