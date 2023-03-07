package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Wait the specified seconds
 *   <li>Drive to the dock and will stop when either:
 *       <ul>
 *         <li>specified distance is reached OR
 *         <li>ramp climb has been detected and leveling off
 *       </ul>
 *   <li>Balance on the dock
 *   <li>Rotate 90 degrees
 * </ol>
 *
 * @see bhs.devilbotz.commands.auto.DriveStraightToDock
 * @see bhs.devilbotz.commands.auto.BalancePID
 * @see bhs.devilbotz.commands.auto.RotateDegrees
 */
public class DockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Dock & Engage routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param maxDistance the max distance to travel before assuming we are on the dock. Negative
   *     indicates move backwards. (in meters)
   */
  public DockAndEngage(DriveTrain drivetrain, double delay, double maxDistance) {
    super();
    addCommands(Commands.waitSeconds(delay));
    addCommands(new DriveStraightToDock(drivetrain, maxDistance));
    addCommands(new BalancePID(drivetrain));
    addCommands(new RotateDegrees(drivetrain, 90));
    addCommands(drivetrain.stop());
  }
}
