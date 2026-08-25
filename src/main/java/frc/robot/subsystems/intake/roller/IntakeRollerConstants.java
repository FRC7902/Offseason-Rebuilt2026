package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class IntakeRollerConstants {

  public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1); // TODO
  public static final int CAN_ID = 5; // TODO

  public static final AngularVelocity TOLERANCE = RPM.of(10); // TODO

  public static final AngularVelocity INTAKE_SPEED = RPM.of(750); // TODO

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(0.00016541, 0, 0) // TODO
          .withSimClosedLoopController(0.00016541, 0, 0) // TODO
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4))) // TODO
          .withIdleMode(MotorMode.COAST) // TODO
          .withTelemetry(
              "IntakeRollerMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("IntakeRollerMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false) // TODO
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)) // TODO
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)); // TODO

  public static final FlyWheelConfig FLYWHEEL_CONFIG =
      new FlyWheelConfig()
          .withDiameter(Inches.of(4)) // TODO
          .withTelemetry("IntakeRollerMech", TelemetryVerbosity.HIGH)
          .withSpeedometerSimulation(RPM.of(750)); // TODO
}
