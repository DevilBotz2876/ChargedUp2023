package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.lib.CommunityLocation;
import bhs.devilbotz.subsystems.DriveTrain;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Mobility, Dock and Engage" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Load the proper trajectory that will move the robot to in front of the dock (wall or
   *       human side)
   *   <li>Translate the path depending on alliance (red or blue)
   *   <li>Execute the requested trajectory
   *   <li>Dock and Engage
   * </ol>
   *
   * <p>Trajectories
   *
   * <ul>
   *   <li>Human Side: <img src="./doc-files/MDE_Human_Blue_Trajectory.png" alt="MDE Start/End
   *       Trajectory for Human Side"/>
   *   <li>Wall Side: <img src="./doc-files/MDE_Wall_Blue_Trajectory.png" alt=MDE Start/End
   *       Trajectory for Wall Side/>
   * </ul>
   *
   * @param drivetrain the DriveTrain object
   * @param startLocation location on the field we are starting from (wall or human)
   * @param alliance the alliance robot is on (red or blue). The requested path is translated as
   *     needed.
   * @param startAngle the starting angle of the robot. During balancing this command will try to
   *     stay square while approaching the balance station
   * @see com.pathplanner.lib.PathPlanner
   * @see bhs.devilbotz.subsystems.DriveTrain#followTrajectoryCommand(PathPlannerTrajectory,
   *     boolean, boolean)
   * @see bhs.devilbotz.commands.auto.DockAndEngage
   */
  public MobilityDockAndEngage(
      DriveTrain drivetrain,
      CommunityLocation startLocation,
      Alliance alliance,
      double startAngle) {
    super();

    PathPlannerTrajectory path = null;
    // Figure out which path to load based on alliance color and autoMode
    String pathName;
    switch (startLocation) {
      case HUMAN:
        pathName = "MobilityBlueHumanSideToDock";
        break;

      case WALL:
        pathName = "MobilityBlueWallSideToDock";
        break;

      default:
        return;
    }
    path = PathPlanner.loadPath(pathName, new PathConstraints(2.5, 2));
    // set the velocity at the end of the path fast enough to dock
    path.getEndState().velocityMetersPerSecond = 1.5;

    addCommands(CommandDebug.start());
    addCommands(drivetrain.followTrajectoryCommand(path, true, false));
    // Since the trajectory causes the robot to turn 180 degrees, we set the targetAngle accordingly
    addCommands(new DockAndEngage(drivetrain, 2, startAngle - 180));
    addCommands(CommandDebug.end());
  }
}
