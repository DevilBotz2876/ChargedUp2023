package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import java.util.Map;
import java.util.function.Supplier;

/**
 * This command will:
 *
 * <ol>
 *   <li>Move the arm the best position for cone vs cube
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.PickupFromGround
 */
public class PrepareForScore extends SelectCommand {
  public PrepareForScore(
      Arm arm, double targetPosition, Gripper gripper, Supplier<Object> selector) {
    // Maps selector values to commands
    super(
        Map.ofEntries(
            Map.entry(GamePieceTypes.CONE, new ArmToPosition(arm, targetPosition, gripper)),
            Map.entry(
                GamePieceTypes.CUBE,
                new ArmToPosition(
                    arm, targetPosition + ArmConstants.POSITION_CUBE_DELTA, gripper))),
        selector);
  }
}
