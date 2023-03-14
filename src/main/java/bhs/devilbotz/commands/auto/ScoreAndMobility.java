package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Auto Score
 *   <li>Move Back to make clearance for the arm
 *   <li>Put the arm down
 *   <li>Mobility
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.AutoScore
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightPID
 * @see bhs.devilbotz.commands.auto.Mobility
 */
public class ScoreAndMobility extends SequentialCommandGroup {
  public ScoreAndMobility(Arm arm, DriveTrain drivetrain, double distance, Gripper gripper) {
    super();

    addCommands(CommandDebug.start());
    addCommands(new AutoScore(arm, drivetrain, gripper));
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(drivetrain.stop());
    addCommands(new ArmDown(arm, gripper));
    // Since we are scoring, we always want to drive backwards in the end. Always set negative
    // distance in case the driver forgets
    addCommands(new Mobility(drivetrain, -(Math.abs(distance))));
    addCommands(CommandDebug.end());
  }
}
