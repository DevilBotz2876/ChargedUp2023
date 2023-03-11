package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmDown;
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
 *   <li>Wait the specified seconds
 *   <li>Drive straight the specified distance
 * </ol>
 *
 * @see bhs.devilbotz.commands.auto.DriveStraightPID
 * @see bhs.devilbotz.commands.auto.RotateDegrees
 */
public class ScoreMobilityDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   */
  //Arm arm, Gripper gripper, DriveTrain drivetrain
  //DriveTrain drivetrain, double delay, double distance

  public ScoreMobilityDockAndEngage(Arm arm, DriveTrain drivetrain, double delay, Gripper gripper) {
    super();

    addCommands(Commands.waitSeconds(delay));
    addCommands(new DriveStraightPID(drivetrain, -1));
    addCommands(new ArmToPosition(arm, ArmConstants.POSITION_TOP));
    addCommands(new DriveStraightPID(drivetrain, 1));
    addCommands(new ArmDown(arm, gripper));
    addCommands(new GripperOpen(gripper));
    addCommands(new DriveStraightPID(drivetrain, -1));
    addCommands(new ArmDown(arm, gripper));
    addCommands(new DriveStraightToDock(drivetrain, 1));
    addCommands(drivetrain.stop());
  }
}