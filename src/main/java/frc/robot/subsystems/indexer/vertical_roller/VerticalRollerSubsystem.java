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
  private static VerticalRollerSubsystem m_instance;

  private final SparkMax m_verticalRollerMotor;
  private final SmartMotorControllerConfig m_motorConfig;
  private final SmartMotorController m_motor;
  private final FlyWheel m_verticalRoller;

  private VerticalRollerSubsystem() {
    m_verticalRollerMotor = new SparkMax(VerticalRollerConstants.CAN_ID, MotorType.kBrushless);
    m_motorConfig = VerticalRollerConstants.SMC_CONFIG.withSubsystem(this);
    m_motor = new SparkWrapper(m_verticalRollerMotor, VerticalRollerConstants.MOTOR, m_motorConfig);
    m_verticalRoller = new FlyWheel(VerticalRollerConstants.FLY_WHEEL_CONFIG, m_motor);
  }

  public static VerticalRollerSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new VerticalRollerSubsystem();
    }
    return m_instance;
  }

  public AngularVelocity getVelocity() {
    return m_verticalRoller.getSpeed();
  }

  private AngularVelocity getVelocitySetpoint() {
    return m_verticalRoller.getMechanismSetpointVelocity().orElse(RPM.of(0));
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_verticalRoller.runTo(speed, VerticalRollerConstants.TOLERANCE);
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_verticalRoller.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_verticalRoller.runTo(speed, VerticalRollerConstants.TOLERANCE);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_verticalRoller.set(dutyCycle);
  }

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
