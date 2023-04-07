package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Score, Mobility, Dock and Engage" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Auto Score
   *   <li>Move Back to make clearance for the arm
   *   <li>Put the arm down
   *   <li>Dock and Engage
   * </ol>
   *
   * <p><i>Note: The robot is assumed to be:</i>
   *
   * <ul>
   *   <li>in front of the charge station
   *   <li>the arm facing and aligned with the scoring position
   *   <li>pushed up against the scoring bumpers
   *   <li>pre-loaded with a game piece (cone)
   * </ul>
   *
   * @param arm the Arm object
   * @param drivetrain the DriveTrain object
   * @param maxDistance the distance to travel. Negative indicates move backwards. (in meters)
   * @param gripper the gripper object
   * @param startAngle the orientation of the robot at the start of the match
   * @see bhs.devilbotz.commands.assist.AutoScore
   * @see bhs.devilbotz.commands.auto.DockAndEngage
   */
  public ScoreDockAndEngage(
      Arm arm, DriveTrain drivetrain, double maxDistance, Gripper gripper, double startAngle) {
    super();

    addCommands(CommandDebug.start());
    // Execute the auto score routine
    addCommands(new AutoScore(arm, drivetrain, gripper));
    addCommands(
        new ParallelCommandGroup(
            // Move arm down but not too low to hit the ramp/ground
            new ArmToPosition(arm, 150, gripper),
            // Since we are scoring, we always want to drive backwards in the end. Always set
            // negative distance in case the driver forgets
            new DockAndEngage(drivetrain, -(Math.abs(maxDistance)), startAngle)));
    // Lower the arm all the way for stability
    addCommands(new ArmDown(arm, gripper));
    addCommands(drivetrain.stopCommand());
    addCommands(CommandDebug.end());
  }
}
