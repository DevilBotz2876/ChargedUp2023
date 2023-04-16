package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupFromGround extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Pickup From Ground" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>Close the gripper
   *   <li>Drive backwards the specified distance (to prevent a stuck arm)
   *   <li>Raise the arm a safe driving position
   * </ol>
   *
   * <i>Note: This command assumes the robot and arm are already positioned to to pickup a
   * piece!</i>
   *
   * @param arm the Arm object
   * @param drivetrain the Drivetain object
   * @param gripper the Gripper object
   * @see bhs.devilbotz.commands.assist.PrepareForGroundPickup
   */
  public PickupFromGround(Arm arm, Gripper gripper, Drive drive) {
    super();
    addCommands(CommandDebug.start());
    // Close the gripper
    addCommands(new GripperClose(gripper));
    // Drive backward and raise arm at the same time
    addCommands(
        new ParallelCommandGroup(
            // Drive backward
            new SequentialCommandGroup(new DriveStraightPID(drive, -0.3), drive.stopCommand()),
            // Raise Arm to safe driving position
            new ArmToPosition(arm, ArmConstants.POSITION_DRIVE, gripper)));
    addCommands(CommandDebug.end());
  }
}
