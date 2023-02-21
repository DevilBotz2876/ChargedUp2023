package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class WallZoneMobilityDockAndEngage extends SequentialCommandGroup {
  // This will load the file "MobilityBlueWallSideToDock.path" and generate it with a max velocity
  // of 1 m/s and a
  // max acceleration of 0.5 m/s^2
  PathPlannerTrajectory WallZoneMobilityDockAndEngage =
      PathPlanner.loadPath("WallZoneMobilityDockAndEngage", new PathConstraints(1, 0.5));

  /**
   * The constructor for the balance auto command.
   *
   * @param drive The drive train subsystem.
   */
  public WallZoneMobilityDockAndEngage(DriveTrain drive) {
    addCommands(drive.followTrajectoryCommand(WallZoneMobilityDockAndEngage, true));
  }
}
