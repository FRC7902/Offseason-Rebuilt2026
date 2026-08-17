package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;

public class IntakeSystem extends SubsystemBase {

    private final LinearIntakeSubsystem m_linearIntake;
    private final IntakeRollerSubsystem m_intakeRoller;

    public IntakeSystem() {
        m_linearIntake = new LinearIntakeSubsystem();
        m_intakeRoller = new IntakeRollerSubsystem();
    }
}
