package frc.robot.utils;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

public class AutoHelper {
  private static final StructArrayPublisher<Pose2d> autoPathPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/PathPlanner/selectedAuto", Pose2d.struct)
          .publish();

  public static void publishAutoPath(String autoName) {
    try {
      List<Pose2d> combinedPoses = new ArrayList<>();

      boolean redAlliance =
          DriverStation.getAlliance()
              .map(alliance -> alliance == DriverStation.Alliance.Red)
              .orElse(false);

      for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {

        PathPlannerPath displayPath = redAlliance ? path.mirrorPath() : path;
        combinedPoses.addAll(displayPath.getPathPoses());
      }

      autoPathPublisher.set(combinedPoses.toArray(Pose2d[]::new));
    } catch (Exception e) {
      DriverStation.reportError("Could not load auto paths: " + e.getMessage(), e.getStackTrace());
    }
  }
}
