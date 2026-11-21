package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.linear.LinearIntakeConstants;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerConstants;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;

public class IntakeSystem extends SubsystemBase {
  private final LinearIntakeSubsystem m_linearIntake;
  private final IntakeRollerSubsystem m_intakeRoller;

  public IntakeSystem(LinearIntakeSubsystem linearIntake, IntakeRollerSubsystem intakeRoller) {
    m_linearIntake = linearIntake;
    m_intakeRoller = intakeRoller;
  }

  public Command extend() {
    return m_linearIntake.setHeight(LinearIntakeConstants.FULLY_EXTENDED);
  }

  public Command extendAndIntake() {
    return Commands.parallel(
        m_linearIntake.setHeight(LinearIntakeConstants.FULLY_EXTENDED),
        m_intakeRoller.setSpeed(IntakeRollerConstants.INTAKE_DUTY_CYCLE));
  }

  public Command extendAndOuttake() {
    return Commands.parallel(
        m_linearIntake.setHeight(LinearIntakeConstants.FULLY_EXTENDED),
        m_intakeRoller.setSpeed(-IntakeRollerConstants.INTAKE_DUTY_CYCLE));
  }

  public Command stop() {
    return Commands.sequence(
        m_linearIntake.setHeight(LinearIntakeConstants.MIDPOINT_DISTANCE), m_intakeRoller.stop());
  }

  public Command shuffle() {
    return Commands.sequence(
            m_linearIntake.setHeight(LinearIntakeConstants.MIDPOINT_DISTANCE).withTimeout(1),
            m_linearIntake.setHeight(LinearIntakeConstants.FULLY_EXTENDED).withTimeout(1))
        .repeatedly()
        .alongWith(m_intakeRoller.setSpeed(IntakeRollerConstants.INTAKE_DUTY_CYCLE));
  }
}
