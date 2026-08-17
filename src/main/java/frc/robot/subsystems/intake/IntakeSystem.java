package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;

public class IntakeSystem {

    private final LinearIntakeSubsystem m_linearIntake;
    private final IntakeRollerSubsystem m_intakeRoller;

    public IntakeSystem(LinearIntakeSubsystem linearIntake, IntakeRollerSubsystem intakeRoller) {
        m_linearIntake = linearIntake;
        m_intakeRoller = intakeRoller;
    }
}
