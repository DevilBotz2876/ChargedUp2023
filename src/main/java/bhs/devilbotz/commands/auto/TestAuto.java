package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {
  // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a
  // max acceleration of 3 m/s^2
  PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3));

  /**
   * The constructor for the balance auto command.
   *
   * @param drive The drive train subsystem.
   */
  public TestAuto(DriveTrain drive) {
    addCommands(drive.followTrajectoryCommand(testPath, true));
  }
}
