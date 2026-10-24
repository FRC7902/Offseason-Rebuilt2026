package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  private static IntakeRollerSubsystem m_instance;

  /** PWM motor controller for the intake. */
  private final TalonFX m_rollerMotor;

  /** Constructs the intake subsystem and initializes the motor controller. */
  private IntakeRollerSubsystem() {

    m_rollerMotor = new TalonFX(IntakeRollerConstants.CAN_ID);
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
