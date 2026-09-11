package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;

public class LimelightWrapper extends Limelight {

  private final LimelightPoseEstimator m_poseEstimator;

  public LimelightWrapper(String name) {
    super(name);

    m_poseEstimator = createPoseEstimator(EstimationMode.MEGATAG2);
  }

  public void updateLocalization() {
    getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(
                    Radians.zero(),
                    Radians.zero(),
                    SwerveDriveSubsystem.getInstance().getGyroAngle()),
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();
  }
}
