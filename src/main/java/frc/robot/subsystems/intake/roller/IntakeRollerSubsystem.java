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

  private final TalonFX m_rollerMotor;

  private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  private IntakeRollerSubsystem() {
    m_rollerMotor = new TalonFX(IntakeRollerConstants.CAN_ID);

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    m_currentLimits
        .withSupplyCurrentLowerLimit(Amps.of(40))
        .withSupplyCurrentLimit(Amps.of(40))
        .withSupplyCurrentLimitEnable(true);

    m_currentLimits
        .withStatorCurrentLimit(Amps.of(40))
        .withStatorCurrentLimitEnable(true);

    toConfigure.CurrentLimits = m_currentLimits;

    m_rollerMotor.getConfigurator().apply(toConfigure);
  }

  public static IntakeRollerSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeRollerSubsystem();
    }
    return m_instance;
  }

  public Command setSpeed(double speed) {
    return this.runOnce(() -> m_rollerMotor.set(speed));
  }

  public Command stop() {
    return this.runOnce(() -> m_rollerMotor.stopMotor());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRollerMech/Duty Cycle", m_rollerMotor.get());
  }
}
