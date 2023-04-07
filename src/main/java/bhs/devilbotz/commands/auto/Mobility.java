package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Mobility" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Drive the specified distance in a straight line
   * </ol>
   *
   * @param drivetrain the DriveTrain object
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   * @see bhs.devilbotz.commands.drivetrain.DriveStraightPID
   */
  public Mobility(DriveTrain drivetrain, double distance) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightPID(drivetrain, distance, 1));
    addCommands(drivetrain.stopCommand());
    addCommands(CommandDebug.end());
  }
}
