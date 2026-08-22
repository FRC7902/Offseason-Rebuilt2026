package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.linear.LinearIntakeConstants;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerConstants;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;

public class IntakeSystem {

  private final LinearIntakeSubsystem m_linearIntake;
  private final IntakeRollerSubsystem m_intakeRoller;

  public IntakeSystem(LinearIntakeSubsystem linearIntake, IntakeRollerSubsystem intakeRoller) {
    m_linearIntake = linearIntake;
    m_intakeRoller = intakeRoller;
  }
  /**
   * Creates a command that extends the linear intake to its fully extended setpoint while running
   * the intake rollers.
   *
   * @return command that runs until the linear intake is fully extended
   */
  public Command extendAndIntake() {
    return Commands.parallel(
        m_linearIntake.setHeight(LinearIntakeConstants.FULLY_EXTENDED),
        m_intakeRoller.setVelocity(IntakeRollerConstants.INTAKE_SPEED));
  }

  /**
   * Creates a command that retracts the linear intake to the midpoint setpoint, then stops the
   * intake rollers once midpoint is reached.
   *
   * @return command that runs until the linear intake is retracted and the rollers are stopped
   */
  public Command retractThenStopIntake() {
    return Commands.sequence(
        m_linearIntake.setHeight(LinearIntakeConstants.MIDPOINT_DISTANCE), m_intakeRoller.stop());
  }

  /**
   * Creates a command that shuffles the hopper by repeatedly moving the linear intake in and out.
   *
   * <p>If the linear intake starts farther extended than midpoint, the command runs the intake
   * rollers until the intake retracts back to midpoint. Otherwise, intake rollers should not be
   * running.
   *
   * @return command that runs indefinitely until interrupted, shuffling the hopper
   */
  public Command shuffleHopper() {
    // TODO: Use constants for shuffling positions
    throw new UnsupportedOperationException("Not yet implemented.");
  }
}
