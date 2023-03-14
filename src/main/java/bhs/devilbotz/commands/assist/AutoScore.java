package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>move the robot back enough to give the arm some clearance
 *   <li>move the arm to the top scoring position
 *   <li>move the robot back to the original position
 *   <li>move the arm down slightly
 *   <li>open the gripper
 * </ol>
 */
public class AutoScore extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param arm the Arm object
   * @param drivetrain the Drivetain object
   * @param gripper the Gripper object
   */
  // Arm arm, Gripper gripper, DriveTrain drivetrain
  // DriveTrain drivetrain, double delay, double distance

  public AutoScore(Arm arm, DriveTrain drivetrain, Gripper gripper) {
    super();
    addCommands(CommandDebug.start());
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(drivetrain.stop());
    addCommands(new ArmToPosition(arm, ArmConstants.POSITION_TOP, gripper));
    addCommands(new DriveStraightPID(drivetrain, DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(drivetrain.stop());
    addCommands(new ArmMoveDistance(arm, ArmConstants.POSITION_SCORING_DELTA, gripper));
    addCommands(new GripperOpen(gripper));
    addCommands(CommandDebug.end());
  }
}
