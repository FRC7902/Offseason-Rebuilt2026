package frc.robot.subsystems.indexer.feeder;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX m_feederMotor;
    private final SmartMotorControllerConfig m_motorConfig;
    private final SmartMotorController m_motor;
    private final FlyWheel m_feeder;

    public FeederSubsystem() {
        m_feederMotor = new TalonFX(FeederConstants.CAN_ID);
        m_motorConfig = FeederConstants.SMC_CONFIG.withSubsystem(this);
        m_motor = new TalonFXWrapper(m_feederMotor, FeederConstants.MOTOR, m_motorConfig);
        m_feeder = new FlyWheel(FeederConstants.FLY_WHEEL_CONFIG, m_motor);
    }

    /**
     * Returns the current feeder angular velocity as measured by the motor
     * encoder.
     *
     * @return Current feeder speed.
     */
    public AngularVelocity getVelocity() {
        return m_feeder.getSpeed();
    }

    /**
     * Runs the feeder at a fixed target velocity. The closed-loop
     * controller
     * and feedforward maintain this speed continuously until the command ends.
     *
     * @param speed Desired angular velocity at the feeder (after gearing).
     * @return A command that holds the given speed while scheduled.
     */
    public Command setVelocity(AngularVelocity speed) {
        return m_feeder.run(speed);
    }

    /**
     * Drives the feeder in open-loop at a fixed duty cycle.
     *
     * @param dutyCycle Output fraction in [-1, 1].
     * @return A command that applies the given duty cycle while scheduled.
     */
    public Command setDutyCycle(double dutyCycle) {
        return m_feeder.set(dutyCycle);
    }

    /**
     * Supplier-based velocity command, suitable for joystick-driven or
     * dashboard-driven speed control where the setpoint changes each loop
     * iteration.
     *
     * @param speed Supplier of the desired angular velocity.
     * @return A command that continuously polls the supplier and updates the
     *         setpoint.
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return m_feeder.run(speed);
    }

    /**
     * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)}
     * for open-loop use cases.
     *
     * @param dutyCycle Supplier of the output fraction in [-1, 1].
     * @return A command that continuously polls the supplier.
     */
    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return m_feeder.set(dutyCycle);
    }

    @Override
    public void periodic() {
        m_feeder.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_feeder.simIterate();
    }
}