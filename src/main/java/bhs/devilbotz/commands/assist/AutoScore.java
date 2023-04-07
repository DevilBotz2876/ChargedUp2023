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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoScore extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the "Auto Score" routine
   *
   * <p>This command will:
   *
   * <ol>
   *   <li>move the robot back enough to give the arm some clearance
   *   <li>move the arm to the top scoring position
   *   <li>move the robot back to the original position
   *   <li>move the arm down slightly
   *   <li>open the gripper
   *   <li>wait a fixed amount of time (for gripper to open and piece to drop)
   * </ol>
   *
   * <i>Note: This command assumes the robot and arm are already positioned and lined up with the
   * scoring target</i>
   *
   * @param arm the Arm object
   * @param drivetrain the Drivetain object
   * @param gripper the Gripper object
   */
  public AutoScore(Arm arm, DriveTrain drivetrain, Gripper gripper) {
    super();
    addCommands(CommandDebug.start());
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL, .9),
                drivetrain.stopCommand()),
            new ArmToPosition(arm, ArmConstants.POSITION_TOP, gripper)));
    addCommands(new DriveStraightPID(drivetrain, DriveConstants.POSITION_DRIVE_FROM_PORTAL, .5));
    addCommands(drivetrain.stopCommand());
    addCommands(new WaitCommand(0.8));
    addCommands(new ArmMoveDistance(arm, ArmConstants.POSITION_SCORING_DELTA - 10, gripper));
    addCommands(new GripperOpen(gripper));
    addCommands(CommandDebug.end());
  }
}
