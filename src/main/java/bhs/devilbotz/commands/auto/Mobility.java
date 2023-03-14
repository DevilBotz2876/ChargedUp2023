package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Drive straight the specified distance
 * </ol>
 *
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightPID
 */
public class Mobility extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param drivetrain the DriveTrain object
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   */
  public Mobility(DriveTrain drivetrain, double distance) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightPID(drivetrain, distance));
    addCommands(CommandDebug.end());
  }
}
