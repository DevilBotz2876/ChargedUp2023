package bhs.devilbotz.commands.assist;

import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepareForGroundPickup extends SequentialCommandGroup {
  public PrepareForGroundPickup(Arm arm, Gripper gripper) {
    super(new ArmDown(arm, gripper), new GripperOpen(gripper));
  }
}
