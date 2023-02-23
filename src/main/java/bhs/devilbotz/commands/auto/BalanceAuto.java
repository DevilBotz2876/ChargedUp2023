package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command is a sequential command group that runs the balance command, and other commands that
 * we may add during the season.
 *
 * @since 1/18/2023
 */
public class BalanceAuto extends SequentialCommandGroup {
  private final DriveTrain drive;

  /**
   * The constructor for the balance auto command.
   *
   * @param drive The drive train subsystem.
   */
  public BalanceAuto(DriveTrain drive) {
    this.drive = drive;

    addCommands(new BalancePID(drive));
  }
}
