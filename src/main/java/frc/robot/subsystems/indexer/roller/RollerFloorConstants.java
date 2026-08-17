package frc.robot.subsystems.indexer.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RollerFloorConstants {

    public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final int CAN_ID = 1;

    public static final SmartMotorControllerConfig SMC_CONFIG = new SmartMotorControllerConfig()
            .withClosedLoopController(0.00016541, 0, 0)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("RollerFloorMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withMomentOfInertia(Inches.of(4), Pounds.of(1))
            .withControlMode(ControlMode.CLOSED_LOOP);

    public static final FlyWheelConfig FLY_WHEEL_CONFIG = new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withTelemetry("RollerFloorMech", TelemetryVerbosity.HIGH)
            .withSpeedometerSimulation(RPM.of(750));
}
