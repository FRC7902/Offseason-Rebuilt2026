package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.linear.LinearIntakeConstants;
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
    // TODO: Use a constant for the intake roller speed
    throw new UnsupportedOperationException("Not yet implemented.");
  }

  /**
   * Creates a command that retracts the linear intake to the midpoint setpoint, then stops the
   * intake rollers once midpoint is reached.
   *
   * @return command that runs until the linear intake is retracted and the rollers are stopped
   */
  public Command retractThenStopIntake() {
    throw new UnsupportedOperationException("Not yet implemented.");
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
    return Commands.sequence(
      new ConditionalCommand(
        Commands.sequence( // first shuffle, half to past half
          m_linearIntake.setHeight(LinearIntakeConstants.HALF_EXTENDED),
          m_linearIntake.setHeight(LinearIntakeConstants.SHUFFLE_MIDPOINT),
          m_linearIntake.setHeight(LinearIntakeConstants.HALF_EXTENDED),
          m_linearIntake.setHeight(LinearIntakeConstants.NEAR_FULLY_RETRACTED)
        ),
        Commands.sequence( 
          new ConditionalCommand(
            Commands.sequence( // second shuffle, quarter to half
              m_linearIntake.setHeight(LinearIntakeConstants.NEAR_FULLY_RETRACTED),
              m_linearIntake.setHeight(LinearIntakeConstants.HALF_EXTENDED),
              m_linearIntake.setHeight(LinearIntakeConstants.NEAR_FULLY_RETRACTED),
              m_linearIntake.setHeight(LinearIntakeConstants.HALF_EXTENDED),
              m_linearIntake.setHeight(LinearIntakeConstants.FULLY_RETRACTED)
            ),
            Commands.sequence( // last repeating shuffle, retracted to quarter
              m_linearIntake.setHeight(LinearIntakeConstants.FULLY_RETRACTED),
              m_linearIntake.setHeight(LinearIntakeConstants.NEAR_FULLY_RETRACTED)
            ),
            () -> m_linearIntake.getHeight().gte(LinearIntakeConstants.NEAR_FULLY_RETRACTED)
          )
        ),
        () -> m_linearIntake.getHeight().gte(LinearIntakeConstants.HALF_EXTENDED)
      )
    ).repeatedly();
  }
}
