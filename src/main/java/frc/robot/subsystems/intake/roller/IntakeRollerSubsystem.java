package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  private static IntakeRollerSubsystem m_instance;

  /** PWM motor controller for the intake. */
  private final TalonFX m_rollerMotor;

  private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  /** Constructs the intake subsystem and initializes the motor controller. */
  private IntakeRollerSubsystem() {
    m_rollerMotor = new TalonFX(IntakeRollerConstants.CAN_ID);

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    m_currentLimits
        .withSupplyCurrentLowerLimit(Amps.of(40)) // Default limit of 70 A
        .withSupplyCurrentLimit(Amps.of(40)) // Reduce the limit to 40 A if we've limited to 70 A...
        .withSupplyCurrentLimitEnable(true); // And enable it

    m_currentLimits
        .withStatorCurrentLimit(Amps.of(40)) // Limit stator current to 120 A
        .withStatorCurrentLimitEnable(true); // And enable it

    toConfigure.CurrentLimits = m_currentLimits;

    m_rollerMotor.getConfigurator().apply(toConfigure);
  }

  public static IntakeRollerSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeRollerSubsystem();
    }
    return m_instance;
  }

  /**
   * Sets the intake motor speed.
   *
   * @param speed the desired motor output (-1.0 to 1.0)
   */
  public Command setSpeed(double speed) {
    return this.runOnce(() -> m_rollerMotor.set(speed));
  }

  /** Stops the intake motor. */
  public Command stop() {
    return this.runOnce(() -> m_rollerMotor.stopMotor());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRollerMech/Duty Cycle", m_rollerMotor.get());
  }
}
