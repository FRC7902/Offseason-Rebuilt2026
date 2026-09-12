package frc.robot.subsystems.intake.linear;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class LinearIntakeSubsystem extends SubsystemBase {
  private final TalonFX m_linearIntakeMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final Elevator m_linearIntake;

  private final DigitalInput m_leftExtendedLimitSwitch;
  private final DigitalInput m_leftRetractedLimitSwitch;
  private final DigitalInput m_rightExtendedLimitSwitch;
  private final DigitalInput m_rightRetractedLimitSwitch;

  private final Trigger m_leftExtendedTrigger;
  private final Trigger m_rightExtendedTrigger;
  private final Trigger m_leftRetractedTrigger;
  private final Trigger m_rightRetractedTrigger;

  private static LinearIntakeSubsystem m_instance;

  private LinearIntakeSubsystem() {
    m_linearIntakeMotor = new TalonFX(LinearIntakeConstants.CAN_ID);
    m_motorConfig = LinearIntakeConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_linearIntakeMotor, LinearIntakeConstants.MOTOR, m_motorConfig);
    m_linearIntake = new Elevator(LinearIntakeConstants.ELEVATOR_CONFIG, m_motor);

    m_leftExtendedLimitSwitch =
        new DigitalInput(LinearIntakeConstants.LEFT_EXTENDED_LIMIT_SWITCH_DIO);
    m_leftRetractedLimitSwitch =
        new DigitalInput(LinearIntakeConstants.LEFT_RETRACTED_LIMIT_SWITCH_DIO);
    m_rightExtendedLimitSwitch =
        new DigitalInput(LinearIntakeConstants.RIGHT_EXTENDED_LIMIT_SWITCH_DIO);
    m_rightRetractedLimitSwitch =
        new DigitalInput(LinearIntakeConstants.RIGHT_RETRACTED_LIMIT_SWITCH_DIO);

    m_leftExtendedTrigger = new Trigger(this::getLeftExtendedLimitSwitch);
    m_rightExtendedTrigger = new Trigger(this::getRightExtendedLimitSwitch);
    m_leftRetractedTrigger = new Trigger(this::getLeftRetractedLimitSwitch);
    m_rightRetractedTrigger = new Trigger(this::getRightRetractedLimitSwitch);

    m_leftExtendedTrigger.onTrue(Commands.runOnce(this::setEncoderPositionExtended));
    m_rightExtendedTrigger.onTrue(Commands.runOnce(this::setEncoderPositionExtended));
    m_leftRetractedTrigger.onTrue(Commands.runOnce(this::setEncoderPositionRetracted));
    m_rightRetractedTrigger.onTrue(Commands.runOnce(this::setEncoderPositionRetracted));
  }

  public static LinearIntakeSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new LinearIntakeSubsystem();
    }
    return m_instance;
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
   * Supplier-based closed-loop height command. Profiles to the target using the trapezoidal
   * constraints defined in motorConfig, then holds the carriage at that height via the PID
   * controller.
   *
   * @param heightSupplier
   * @return Command that runs until the carriage reaches the target height within the tolerance
   */
  public Command setHeight(Supplier<Distance> heightSupplier) {
    return m_linearIntake.runTo(heightSupplier, LinearIntakeConstants.TOLERANCE);
  }

  /**
   * Returns the current height of the linear intake carriage as measured by the motor encoder.
   *
   * @return Current height of the linear intake carriage.
   */
  public Distance getHeight() {
    return m_linearIntake.getHeight();
  }

  /**
   * Returns the current height setpoint of the linear intake, if one is active. If no setpoint is
   * active, returns zero.
   *
   * @return Current height setpoint of the linear intake.
   */
  private Distance getHeightSetpoint() {
    return m_linearIntake
        .getMechanismSetpoint()
        .map(m_motorConfig::convertFromMechanism)
        .orElse(Meters.zero());
  }

  public Pose3d getPose3d() {
    double angle = Math.toRadians(LinearIntakeConstants.MECHANISM_ANGLE.in(Degrees));
    double distance = LinearIntakeConstants.FULLY_EXTENDED.in(Meters) - getHeight().in(Meters);

    return new Pose3d(
        new Translation3d(distance * Math.cos(angle), 0.0, distance * Math.sin(angle)),
        new Rotation3d());
  }

  /**
   * Stops the linear intake by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  /**
   * Runs a SysId routine on the linear intake mechanism. This command will run a series of
   * quasistatic and dynamic tests, logging the results to the Phoenix SignalLogger. The routine
   * will stop the closed-loop controller before starting and restart it after finishing.
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
                Volts.of(0.5).per(Second),
                // The step voltage output used for dynamic test routines. Defaults to 7 volts
                // if left null.
                Volts.of(1),
                // Safety timeout for the test routine commands. Defaults to 10 seconds if
                // left null.
                Seconds.of(3),
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> m_linearIntakeMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
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

  public boolean getLeftExtendedLimitSwitch() {
    return m_leftExtendedLimitSwitch.get();
  }

  public boolean getRightExtendedLimitSwitch() {
    return m_rightExtendedLimitSwitch.get();
  }

  public boolean getLeftRetractedLimitSwitch() {
    return m_leftRetractedLimitSwitch.get();
  }

  public boolean getRightRetractedLimitSwitch() {
    return m_rightRetractedLimitSwitch.get();
  }

  public boolean getExtendedLimitSwitch() {
    return getLeftExtendedLimitSwitch() || getRightExtendedLimitSwitch();
  }

  public boolean getRetractedLimitSwitch() {
    return getLeftRetractedLimitSwitch() || getRightRetractedLimitSwitch();
  }

  public void setEncoderPositionExtended() {
    m_motor.setEncoderPosition(LinearIntakeConstants.FULLY_EXTENDED);
  }

  public void setEncoderPositionRetracted() {
    m_motor.setEncoderPosition(LinearIntakeConstants.FULLY_RETRACTED);
  }

  @Override
  public void periodic() {
    m_linearIntake.updateTelemetry();

    SmartDashboard.putNumber("LinearIntakeMech/setpoint (m)", getHeightSetpoint().in(Meters));
    SmartDashboard.putNumber("LinearIntakeMech/position (m)", getHeight().in(Meters));

    SmartDashboard.putBoolean(
        "LinearIntakeMech/leftExtendedLimitSwitch", getLeftExtendedLimitSwitch());
    SmartDashboard.putBoolean(
        "LinearIntakeMech/leftRetractedLimitSwitch", getLeftRetractedLimitSwitch());
    SmartDashboard.putBoolean(
        "LinearIntakeMech/rightExtendedLimitSwitch", getRightExtendedLimitSwitch());
    SmartDashboard.putBoolean(
        "LinearIntakeMech/rightRetractedLimitSwitch", getRightRetractedLimitSwitch());
  }

  @Override
  public void simulationPeriodic() {
    m_linearIntake.simIterate();
  }
}
