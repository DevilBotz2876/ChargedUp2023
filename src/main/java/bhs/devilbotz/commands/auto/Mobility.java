package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Wait the specified seconds
 *   <li>Drive straight the specified distance
 * </ol>
 *
 * @see bhs.devilbotz.commands.auto.DriveStraightPID
 * @see bhs.devilbotz.commands.auto.RotateDegrees
 */
public class Mobility extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   */
  public Mobility(DriveTrain drivetrain, double delay, double distance) {
    super();
    addCommands(Commands.waitSeconds(delay));
    addCommands(new DriveStraightPID(drivetrain, distance));
    addCommands(drivetrain.stop());
  }
}
