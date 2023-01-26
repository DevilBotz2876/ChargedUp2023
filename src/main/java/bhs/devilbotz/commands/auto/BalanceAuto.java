package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.Balance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BalanceAuto extends SequentialCommandGroup {
  public BalanceAuto() {

    addCommands(new Balance());
  }
}
