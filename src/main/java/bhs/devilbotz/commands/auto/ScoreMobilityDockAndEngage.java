package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreMobilityDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Score, Mobility, Dock and Engage" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Auto Score
   *   <li>Move Back to make clearance for the arm
   *   <li>Put the arm down
   *   <li>Drive over and past the charge station
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
  public ScoreMobilityDockAndEngage(
      Arm arm, DriveTrain drivetrain, double maxDistance, Gripper gripper, double startAngle) {
    super();

    addCommands(CommandDebug.start());
    // Execute the auto score routine
    addCommands(new AutoScore(arm, drivetrain, gripper));
    // Close the gripper when we are done
    addCommands(new GripperClose(gripper));
    // In parallel, lower the arm to a reasonable/stable position and drive backwards over the
    // charge station to the other side
    addCommands(
        new ParallelCommandGroup(
            // Move arm down but not too low to hit the ramp/ground
            new ArmToPosition(arm, 150, gripper),
            // Since we are scoring, we always want to drive backwards over the charge station.
            // Always set negative distance in case the driver forgets
            new DriveStraightPID(drivetrain, -(Math.abs(maxDistance)), 1)));
    // In parallel, lower the arm the rest of the way down and drive forward to dock and engage
    addCommands(
        new ParallelCommandGroup(
            // Lower the arm all the way for stability
            new ArmDown(arm, gripper),
            // We drove back over the dock, we need to drive forward to the dock
            new DockAndEngage(drivetrain, Math.abs(maxDistance), startAngle)));
    addCommands(drivetrain.stopCommand());
    addCommands(CommandDebug.end());
  }
}
