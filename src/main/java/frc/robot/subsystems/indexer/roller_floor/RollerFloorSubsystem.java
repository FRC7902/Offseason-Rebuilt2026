package frc.robot.subsystems.indexer.roller_floor;

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

public class RollerFloorSubsystem extends SubsystemBase {
  private static RollerFloorSubsystem m_instance;

  private final TalonFX m_rollerFloorMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_rollerFloor;

  private RollerFloorSubsystem() {
    m_rollerFloorMotor = new TalonFX(RollerFloorConstants.CAN_ID);
    m_motorConfig = RollerFloorConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_rollerFloorMotor, RollerFloorConstants.MOTOR, m_motorConfig);
    m_rollerFloor = new FlyWheel(RollerFloorConstants.FLY_WHEEL_CONFIG, m_motor);
  }

  public static RollerFloorSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new RollerFloorSubsystem();
    }
    return m_instance;
  }

  public AngularVelocity getVelocity() {
    return m_rollerFloor.getSpeed();
  }

  private AngularVelocity getVelocitySetpoint() {
    return m_rollerFloor.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_rollerFloor.runTo(speed, RollerFloorConstants.TOLERANCE);
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_rollerFloor.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_rollerFloor.runTo(speed, RollerFloorConstants.TOLERANCE);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_rollerFloor.set(dutyCycle);
  }

  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
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
                (volts) -> m_rollerFloorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));

    Command group =
        Commands.print("Starting SysId!")
            .beforeStarting(Commands.runOnce(m_motor::stopClosedLoopController))
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
            .finallyDo(m_motor::startClosedLoopController)
            .andThen(Commands.print(getName() + " SysId test done."));

    return group.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
  }

  @Override
  public void periodic() {
    m_rollerFloor.updateTelemetry();

    SmartDashboard.putNumber("RollerFloorMech/setpoint (RPM)", getVelocitySetpoint().in(RPM));
    SmartDashboard.putNumber("RollerFloorMech/velocity (RPM)", getVelocity().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    m_rollerFloor.simIterate();
  }
}
