package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.drivetrain.BalancePID;
import bhs.devilbotz.commands.drivetrain.DriveStraightToDock;
import bhs.devilbotz.commands.drivetrain.RotateDegrees;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightToDock
 * @see bhs.devilbotz.commands.drivetrain.BalancePID
 * @see bhs.devilbotz.commands.drivetrain.RotateDegrees
 */
public class DockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Dock and Engage" routine
   *
   * <p>This command will:
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
   * <i>Note: The robot is expected to be positioned squarely in front/back of the charge station at
   * most the maxDistance away from the <b>center</b> of the dock</i>
   *
   * @param drivetrain the DriveTrain object
   * @param maxDistance the max distance to travel before assuming we are on the dock. Negative
   *     indicates move backwards. (in meters)
   * @see bhs.devilbotz.commands.drivetrain.DriveStraightToDock
   * @see bhs.devilbotz.commands.drivetrain.BalancePID
   * @see bhs.devilbotz.commands.drivetrain.RotateDegrees
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
   * Similar to {@link #DockAndEngage(DriveTrain, double)}, with the following difference(s):
   *
   * <ol>
   *   <li>the specified target angle will try to be maintained
   * </ol>
   *
   * @param drivetrain the DriveTrain object
   * @param maxDistance the max distance to travel before assuming we are on the dock. Negative
   *     indicates move backwards. (in meters)
   * @param targetAngle the targetAngle to maintain during the command
   */
  public DockAndEngage(DriveTrain drivetrain, double maxDistance, double targetAngle) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightToDock(drivetrain, maxDistance, targetAngle));
    addCommands(new BalancePID(drivetrain, targetAngle));
    addCommands(new RotateDegrees(drivetrain, 90));
    addCommands(CommandDebug.end());
  }
}
