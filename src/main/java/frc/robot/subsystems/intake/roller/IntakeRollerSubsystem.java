package frc.robot.subsystems.intake.roller;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final TalonFX m_intakeRollerMotor;
    private final SmartMotorControllerConfig m_motorConfig;
    private final SmartMotorController m_motor;
    private final FlyWheel m_intakeRoller;

    public IntakeRollerSubsystem() {
        m_intakeRollerMotor = new TalonFX(IntakeRollerConstants.CAN_ID);
        m_motorConfig = IntakeRollerConstants.SMC_CONFIG.withSubsystem(this);
        m_motor = new TalonFXWrapper(m_intakeRollerMotor, IntakeRollerConstants.MOTOR, m_motorConfig);
        m_intakeRoller = new FlyWheel(IntakeRollerConstants.FLYWHEEL_CONFIG, m_motor);
    }

    /**
     * Returns the current intake roller angular velocity as measured by the motor
     * encoder.
     *
     * @return Current roller speed.
     */
    public AngularVelocity getVelocity() {
        return m_intakeRoller.getSpeed();
    }

    /**
     * Runs the intake rollers at a fixed target velocity. The closed-loop
     * controller
     * and feedforward maintain this speed continuously until the command ends.
     *
     * @param speed Desired angular velocity at the wheel (after gearing).
     * @return A command that runs until the intake roller reaches the target speed
     *         within tolerance.
     */
    public Command setVelocity(AngularVelocity speed) {
        return m_intakeRoller.runTo(speed, IntakeRollerConstants.TOLERANCE);
    }

    /**
     * Drives the intake rollers in open-loop at a fixed duty cycle. Useful for
     * manual
     * tuning or fallback if characterization data is unavailable.
     *
     * @param dutyCycle Output fraction in [-1, 1].
     * @return A command that applies the given duty cycle while scheduled.
     */
    public Command setDutyCycle(double dutyCycle) {
        return m_intakeRoller.set(dutyCycle);
    }

    /**
     * Supplier-based velocity command, suitable for joystick-driven or
     * dashboard-driven
     * speed control where the setpoint changes each loop iteration.
     *
     * @param speed Supplier of the desired angular velocity.
     * @return A command that runs until the intake roller reaches the target speed
     *         within tolerance. The setpoint is continuously polled from the
     *         supplier, allowing for dynamic speed changes.
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return m_intakeRoller.runTo(speed, IntakeRollerConstants.TOLERANCE);
    }

    /**
     * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)}
     * for
     * open-loop use cases.
     *
     * @param dutyCycle Supplier of the output fraction in [-1, 1].
     * @return A command that continuously polls the supplier.
     */
    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return m_intakeRoller.set(dutyCycle);
    }

    @Override
    public void periodic() {
        m_intakeRoller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_intakeRoller.simIterate();
    }
}