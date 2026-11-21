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
  private static LinearIntakeSubsystem m_instance;

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

  public Command setDutyCycle(double dutyCycle) {
    return m_linearIntake.set(dutyCycle);
  }

  public Command setHeight(Distance height) {
    return m_linearIntake.runTo(height, LinearIntakeConstants.TOLERANCE);
  }

  public Command setHeight(Supplier<Distance> heightSupplier) {
    return m_linearIntake.runTo(heightSupplier, LinearIntakeConstants.TOLERANCE);
  }

  public Distance getHeight() {
    return m_linearIntake.getHeight();
  }

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

  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public Command sysId() {
    final VoltageOut m_voltReq = new VoltageOut(0.0);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(

                Volts.of(0.5).per(Second),

                Volts.of(1),

                Seconds.of(3),

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

  }

  @Override
  public void simulationPeriodic() {
    m_linearIntake.simIterate();
  }
}
