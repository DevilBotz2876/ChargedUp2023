package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Move the arm all the way down
 *   <li>Open the gripper
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.PickupFromGround
 */
public class PrepareForGroundPickup extends SequentialCommandGroup {
  public PrepareForGroundPickup(Arm arm, Gripper gripper) {
    super();
    // close the gripper
    addCommands(new GripperClose(gripper));
    // lower arm
    addCommands(new ArmDown(arm, gripper, ArmConstants.POSITION_GRIPPER_CLOSE));
    // open the gripper
    addCommands(new GripperOpen(gripper));
  }
}
