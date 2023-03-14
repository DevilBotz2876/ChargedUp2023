package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Scores, then docks and engages
 * </ol>
 *
 * @see bhs.devilbotz.commands.auto.DriveStraightPID
 * @see bhs.devilbotz.commands.auto.RotateDegrees
 */
public class AutoScore extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   */
  //Arm arm, Gripper gripper, DriveTrain drivetrain
  //DriveTrain drivetrain, double delay, double distance

  public AutoScore(Arm arm, DriveTrain drivetrain, double delay,  Gripper gripper) {
    super();

    addCommands(Commands.waitSeconds(delay));
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(new ArmToPosition(arm, ArmConstants.POSITION_TOP));
    addCommands(new DriveStraightPID(drivetrain, DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(new ArmMoveDistance(arm, -10, gripper));
    addCommands(new GripperOpen(gripper));
  }
}