package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.BalancePID;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BalanceAuto extends SequentialCommandGroup {
  private final DriveTrain drive;

  public BalanceAuto(DriveTrain drive) {
    this.drive = drive;

    addCommands(new BalancePID(drive));
  }
}