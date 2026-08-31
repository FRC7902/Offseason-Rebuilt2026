package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  /** PWM motor controller for the intake. */
  private final PWMTalonFX m_rollerMotor;

  /** Constructs the intake subsystem and initializes the motor controller. */
  public IntakeRollerSubsystem() {

    m_rollerMotor = new PWMTalonFX(IntakeRollerConstants.PWM_ID);
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
