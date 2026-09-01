package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
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

  public Command setAngle(Supplier<Angle> angle) {
    return m_hood.runTo(angle, HoodConstants.TOLERANCE);
  }
  /**
   * Stops the hood by disabling closed-loop control and commanding zero duty cycle.
   *
   * @return A one-shot command that stops the mechanism.
   */
  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  /**
   * Runs a SysId routine on the hood mechanism. This command will run a series of quasistatic and
   * dynamic tests, logging the results to the Phoenix SignalLogger. The routine will stop the
   * closed-loop controller before starting and restart it after finishing.
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
                (volts) -> m_hoodMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));

    Command group =
        Commands.print("Starting SysId!")
            .beforeStarting(Commands.runOnce(m_motor::stopClosedLoopController))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
            .finallyDo(m_motor::startClosedLoopController)
            .andThen(Commands.print(getName() + " SysId test done."));

    return group.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
  }

  public boolean isAtSetpoint() {
    return m_hood
        .getMechanismSetpoint()
        .map(setpoint -> m_hood.isNear(setpoint, HoodConstants.TOLERANCE).getAsBoolean())
        .orElse(false);
  }

  public Angle getAngle() {
    return m_hood.getAngle();
  }

  public Angle getAngleSetpoint() {
    return m_hood.getMechanismSetpoint().orElse(HoodConstants.MIN_ANGLE);
  }

  public Pose3d getPose3d(Pose3d turretPose) {
    return turretPose.transformBy(
        new Transform3d(
            Units.Inches.of(4.559248).in(Units.Meters),
            0.0,
            Units.Inches.of(4.339886).in(Units.Meters),
            new Rotation3d(0.0, getAngle().minus(HoodConstants.MIN_ANGLE).in(Units.Radians), 0.0)));
  }

  @Override
  public void periodic() {
    m_hood.updateTelemetry();

    SmartDashboard.putNumber("HoodMech/setpoint (deg)", getAngleSetpoint().in(Degrees));
    SmartDashboard.putNumber("HoodMech/position (deg)", getAngle().in(Degrees));

    SmartDashboard.putBoolean("HoodMech/isAtSetpoint", isAtSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    m_hood.simIterate();
  }
}
