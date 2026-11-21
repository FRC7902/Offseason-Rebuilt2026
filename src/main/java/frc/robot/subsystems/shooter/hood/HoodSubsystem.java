package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
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
import frc.robot.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.Supplier;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
  private static HoodSubsystem m_instance;

  private final TalonFX m_hoodMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final Arm m_hood;

  private HoodSubsystem() {
    m_hoodMotor = new TalonFX(HoodConstants.CAN_ID);
    m_motorConfig = HoodConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new TalonFXWrapper(m_hoodMotor, HoodConstants.MOTOR, m_motorConfig);
    m_hood = new Arm(HoodConstants.ARM_CONFIG, m_motor);
  }

  public static HoodSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new HoodSubsystem();
    }
    return m_instance;
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_hood.set(dutyCycle);
  }

  public Command setAngle(Angle angle) {
    return setAngle(() -> angle);
  }

  public Command setAngle(Supplier<Angle> angle) {
    return m_hood.run(() -> isInTrenchSafetyZone() ? HoodConstants.MIN_ANGLE : angle.get());
  }

  public boolean isInTrenchSafetyZone() {
    return FieldConstants.isInTrenchHoodSafetyZone(
        SwerveDriveSubsystem.getInstance().getPose().getTranslation());
  }

  public Command stop() {
    return this.runOnce(() -> m_motor.stopClosedLoopController()).andThen(setDutyCycle(0));
  }

  public Command sysId() {
    final VoltageOut m_voltReq = new VoltageOut(0.0);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(

                Volts.of(0.5).per(Seconds),

                Volts.of(0.5),

                Seconds.of(3),

                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> m_hoodMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
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

  public boolean isHoodReadyToShoot() {
    return isAtSetpoint() && !isInTrenchSafetyZone();
  }

  @Override
  public void periodic() {
    m_hood.updateTelemetry();

    boolean inTrenchSafetyZone = isInTrenchSafetyZone();
    if (inTrenchSafetyZone) {
      m_hood.setMechanismPositionSetpoint(HoodConstants.MIN_ANGLE);
    }

    SmartDashboard.putNumber("HoodMech/setpoint (deg)", getAngleSetpoint().in(Degrees));
    SmartDashboard.putNumber("HoodMech/position (deg)", getAngle().in(Degrees));

    SmartDashboard.putBoolean("HoodMech/isAtSetpoint", isAtSetpoint());
    SmartDashboard.putBoolean("HoodMech/isInTrenchSafetyZone", inTrenchSafetyZone);
  }

  @Override
  public void simulationPeriodic() {
    m_hood.simIterate();
  }
}
