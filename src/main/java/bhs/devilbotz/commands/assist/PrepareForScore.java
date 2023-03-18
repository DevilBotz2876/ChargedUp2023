package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import java.util.Map;
import java.util.function.Supplier;

public class PrepareForScore extends SelectCommand {
  /**
   * Creates a sequential command that implements the "Prepare for Score" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Move the arm the best position for scoring a game piece
   * </ol>
   *
   * @param arm the Arm object
   * @param targetPosition the desired arm position
   * @param gripper the Gripper object
   * @param gamePieceTypeSelector selector that returns the desired game piece type
   */
  // TODO: change targetPosition to use ScoreLevels enum, instead
  public PrepareForScore(
      Arm arm, double targetPosition, Gripper gripper, Supplier<Object> gamePieceTypeSelector) {
    // Maps selector values to commands
    super(
        Map.ofEntries(
            Map.entry(GamePieceTypes.CONE, new ArmToPosition(arm, targetPosition, gripper)),
            Map.entry(
                GamePieceTypes.CUBE,
                new ArmToPosition(
                    arm, targetPosition + ArmConstants.POSITION_CUBE_DELTA, gripper))),
        gamePieceTypeSelector);
  }
}
