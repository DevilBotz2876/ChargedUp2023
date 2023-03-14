package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.RobotContainer;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Map;

/**
 * This command will:
 *
 * <ol>
 *   <li>Move the arm down to the best position for cone vs cube
 *   <li>Open the gripper
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.PickupFromGround
 */
public class PrepareForGroundPickup extends SequentialCommandGroup {
  public PrepareForGroundPickup(Arm arm, Gripper gripper, RobotContainer robotContainer) {
    super();
    addCommands(CommandDebug.start());
    // close the gripper
    addCommands(new GripperClose(gripper));

    // lower arm
    addCommands(
        new SelectCommand(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(GamePieceTypes.CONE, new ArmDown(arm, gripper)),
                Map.entry(
                    GamePieceTypes.CUBE,
                    new ArmToPosition(arm, ArmConstants.POSITION_PICKUP_GROUND_CUBE, gripper))),
            robotContainer::getGamePieceType));

    // open the gripper
    addCommands(new GripperOpen(gripper));
    addCommands(CommandDebug.end());
  }
}
