package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.auto.DriveStraightPID;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Close the gripper
 *   <li>Drive backwards the specified distance (to prevent a stuck arm)
 *   <li>Raise the arm a safe driving position
 * </ol>
 *
 * Note: This class assume the robot and arm are already positioned to to pickup a piece!
 *
 * @see bhs.devilbotz.commands.assist.PrepareForGroundPickup
 */
public class PickupFromGround extends SequentialCommandGroup {
  public PickupFromGround(Arm arm, Gripper gripper, DriveTrain drivetrain) {
    super();
    // Close the gripper
    addCommands(new GripperClose(gripper));
    // Drive backward
    addCommands(new DriveStraightPID(drivetrain, -.3));
    addCommands(
        new InstantCommand(
            () -> {
              drivetrain.tankDriveVolts(0, 0);
            }));
    // Raise Arm to safe driving position
    addCommands(
        new ArmToPosition(
            arm, ArmConstants.POSITION_DRIVE, gripper, ArmConstants.POSITION_GRIPPER_CLOSE));
  }
}