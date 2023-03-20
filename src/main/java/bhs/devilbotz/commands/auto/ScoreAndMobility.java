package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAndMobility extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Score and Mobility" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Auto Score
   *   <li>Move Back to make clearance for the arm
   *   <li>Put the arm down
   *   <li>Mobility
   * </ol>
   *
   * <p><i>Note: The robot is assumed to be:</i>
   *
   * <ul>
   *   <li>NOT in front of the charge station
   *   <li>the arm facing and aligned with the scoring position
   *   <li>pushed up against the scoring bumpers
   *   <li>pre-loaded with a game piece (cone)
   * </ul>
   *
   * @param arm the Arm object
   * @param drivetrain the DriveTrain object
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   * @param gripper the gripper object
   * @see bhs.devilbotz.commands.assist.AutoScore
   * @see bhs.devilbotz.commands.auto.Mobility
   */
  public ScoreAndMobility(Arm arm, DriveTrain drivetrain, double distance, Gripper gripper) {
    super();

    addCommands(CommandDebug.start());
    addCommands(new AutoScore(arm, drivetrain, gripper));
    // Move back enough to give clearance for the arm to go down
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL, 1));
    addCommands(
        new ParallelCommandGroup(
            // Move arm all the way down
            new ArmDown(arm, gripper),
            // Since we are scoring, we always want to drive backwards in the end. Always set
            // negative distance in case the driver forgets
            new Mobility(drivetrain, -3)));
    addCommands(CommandDebug.end());
  }
}
