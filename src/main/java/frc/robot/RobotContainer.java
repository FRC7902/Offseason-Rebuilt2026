// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller.RollerFloorSubsystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.intake.linear.LinearIntakeConstants;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer {

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final IndexerBeltSubsystem m_indexerBeltSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final RollerFloorSubsystem m_rollerFloorSubsystem;

  private final LinearIntakeSubsystem m_linearIntakeSubsystem;
  private final IntakeRollerSubsystem m_intakeRollerSubsystem;

  private final FlywheelSubsystem m_flywheelSubsystem;
  private final HoodSubsystem m_hoodSubsystem;
  private final TurretSubsystem m_turretSubsystem;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  private final ShooterSystem m_shooterSystem;

  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final SwerveInputStream driveAngularVelocity;

  private final StructArrayPublisher<Pose3d> posesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/3D/ComponentPoses", Pose3d.struct)
          .publish();

  public RobotContainer() {

    // Start data logging
    DataLogManager.start();
    // Include DriverStation data in the log
    DriverStation.startDataLog(DataLogManager.getLog());

    m_indexerBeltSubsystem = new IndexerBeltSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_rollerFloorSubsystem = new RollerFloorSubsystem();

    m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    m_intakeRollerSubsystem = new IntakeRollerSubsystem();

    m_flywheelSubsystem = new FlywheelSubsystem();
    m_hoodSubsystem = new HoodSubsystem();
    m_turretSubsystem = new TurretSubsystem();

    m_swerveDriveSubsystem = new SwerveDriveSubsystem();
    driveAngularVelocity =
        m_swerveDriveSubsystem
            .getAngularVelocityStream(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                () -> m_driverController.getRawAxis(2))
            .withAllianceRelativeControl();

    m_indexerSystem =
        new IndexerSystem(m_indexerBeltSubsystem, m_feederSubsystem, m_rollerFloorSubsystem);
    m_intakeSystem = new IntakeSystem(m_linearIntakeSubsystem, m_intakeRollerSubsystem);
    m_shooterSystem = new ShooterSystem(m_flywheelSubsystem, m_hoodSubsystem, m_turretSubsystem);

    configureBindings();
  }

  private void configureBindings() {

    m_swerveDriveSubsystem.setDefaultCommand(m_swerveDriveSubsystem.drive(driveAngularVelocity));

    /*
     * TODO: Bind driver controller L2
     * - When held, extend intake and run intake rollers
     * - When held, but not shooting (operator's R2 not held), run indexer to store
     * fuel
     * - When released, retract intake and stop intake rollers
     */

    /*
     * TODO: Bind operator controller R2
     * - When held run shooter, and run indexer to feed balls into shooter when
     * shooter is ready. Stop shooting when released
     * - When held and shooter is ready, shuffle the hopper using the intake. Stop
     * shuffling when released
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void publishComponentPoses() {

    // Use Timer.getFPGATimestamp() to animate the turret angle over a sinusoidal path between 0 and
    // 360 degrees over a period of 5 seconds. Use the current time in seconds as the input to the
    // sine function, and scale the output to the desired range.
    Angle turretAngle =
        Degrees.of(
            TurretConstants.MAX_ANGLE.in(Degrees)
                * (1 + Math.sin(2 * Math.PI * (Timer.getFPGATimestamp() / 10))));

    // Use Timer.getFPGATimestamp() to animate the hood angle over a sinusoidal path between 0 and
    // 20 degrees over a period of 5 seconds. Use the current time in seconds as the input to the
    // sine function, and scale the output to the desired range.
    Angle hoodAngle =
        Degrees.of(
            (HoodConstants.MAX_ANGLE.in(Degrees) / 2)
                * (1 + Math.sin(2 * Math.PI * (Timer.getFPGATimestamp() / 5))));

    Pose3d turretPose =
        new Pose3d(
            new Translation3d(0.144, -0.152, 0.359), new Rotation3d(0, 0, turretAngle.in(Radians)));

    Pose3d hoodPose =
        turretPose.transformBy(
            new Transform3d(
                Inches.of(4.559248).in(Meters),
                0.0,
                Inches.of(4.339886).in(Meters),
                new Rotation3d(0.0, hoodAngle.in(Radians), 0.0)));

    // Linear Intake pose
    double angle = Math.toRadians(LinearIntakeConstants.MECHANISM_ANGLE.in(Degrees));
    // Use Timer.getFPGATimestamp() to animate the linear intake position over a sinusoidal path
    // between 0 and MAX_DISTANCE over a period of 5 seconds. Use the current time in seconds as the
    // input to the sine function, and scale the output to the desired range.
    double distance =
        LinearIntakeConstants.FULLY_EXTENDED.in(Meters)
            - LinearIntakeConstants.FULLY_EXTENDED.in(Meters)
                * (0.5 * Math.sin(Timer.getTimestamp()) + 0.5);

    // Convert distance vector to x and z components based on angle
    double x = distance * Math.cos(angle);
    double z = distance * Math.sin(angle);
    Pose3d linearIntakePose = new Pose3d(new Translation3d(x, 0.0, z), new Rotation3d());

    posesPublisher.set(new Pose3d[] {linearIntakePose, turretPose, hoodPose, new Pose3d()});
  }
}
