package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.auto.DriveStraightPID;
import bhs.devilbotz.commands.auto.RotateDegrees;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Lower the arm slightly
 *   <li>Open the gripper
 *   <li>Drive backwards the specified distance
 *   <li>Get ready for pickup of another piece (close grip, lower arm, open grip)
 *   <li>Rotate 180 degrees
 * </ol>
 *
 * Note: This class assume the robot and arm are already positioned to score!
 *
 * @see bhs.devilbotz.commands.assist.PrepareForScoring
 */
public class AutoScore extends SequentialCommandGroup {
  private final double driveBackDistance = -0.3; // in meters

  public AutoScore(Arm arm, Gripper gripper, DriveTrain drivetrain) {
    super();
    addCommands(CommandDebug.start());
    // Lower the arm slightly
    addCommands(new ArmToPosition(arm, arm.getPosition() - ArmConstants.POSITION_SCORING_DELTA));
    // Open the gripper
    addCommands(new GripperOpen(gripper));
    // backup
    addCommands(new DriveStraightPID(drivetrain, driveBackDistance));
    addCommands(drivetrain.stop());
    // get ready for next piece
    addCommands(new PrepareForGroundPickup(arm, gripper));
    // rotate 180 degrees
    addCommands(new RotateDegrees(drivetrain, 180));
    addCommands(drivetrain.stop());
    addCommands(CommandDebug.end());
  }
}
