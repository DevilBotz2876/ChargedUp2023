package bhs.devilbotz.commands.assist;

import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupFromGround extends SequentialCommandGroup {
  public PickupFromGround(Arm arm, Gripper gripper, DriveTrain drivetrain) {
    super(
        new GripperClose(gripper)
        // TODO: Drive backward
        // TODO: RaiseArmToPosition(Drive)
        );
  }
}
