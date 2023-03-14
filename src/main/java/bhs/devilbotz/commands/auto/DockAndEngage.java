package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.drivetrain.BalancePID;
import bhs.devilbotz.commands.drivetrain.DriveStraightToDock;
import bhs.devilbotz.commands.drivetrain.RotateDegrees;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Drive to the dock and will stop when either:
 *       <ul>
 *         <li>specified distance is reached OR
 *         <li>ramp climb has been detected and leveling off
 *       </ul>
 *   <li>Balance on the dock
 *   <li>Rotate 90 degrees
 * </ol>
 *
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightToDock
 * @see bhs.devilbotz.commands.drivetrain.BalancePID
 * @see bhs.devilbotz.commands.drivetrain.RotateDegrees
 */
public class DockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Dock and Engage routine
   *
   * @param drivetrain the DriveTrain object
   * @param maxDistance the max distance to travel before assuming we are on the dock. Negative
   *     indicates move backwards. (in meters)
   */
  public DockAndEngage(DriveTrain drivetrain, double maxDistance) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightToDock(drivetrain, maxDistance));
    addCommands(new BalancePID(drivetrain));
    addCommands(new RotateDegrees(drivetrain, 90));
    addCommands(CommandDebug.end());
  }

  /**
   * Creates a sequential command that implements the Dock and Engage routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param maxDistance the max distance to travel before assuming we are on the dock. Negative
   *     indicates move backwards. (in meters)
   * @param targetAngle the angle that we need to maintain while docking
   */
  public DockAndEngage(
      DriveTrain drivetrain, double delay, double maxDistance, double targetAngle) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightToDock(drivetrain, maxDistance, targetAngle));
    addCommands(new BalancePID(drivetrain));
    addCommands(new RotateDegrees(drivetrain, 90));
    addCommands(CommandDebug.end());
  }
}
