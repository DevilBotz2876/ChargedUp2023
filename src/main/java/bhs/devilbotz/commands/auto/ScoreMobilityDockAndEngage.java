package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.DriveConstants;
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
 *   <li>Scores, then docks and engages
 * </ol>
 *
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightPID
 * @see bhs.devilbotz.commands.drivetrain.RotateDegrees
 */
public class ScoreMobilityDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param drivetrain the DriveTrain object
   * @param delay the time to wait before starting the command sequence (in seconds)
   * @param distance the distance to travel. Negative indicates move backwards. (in meters)
   */
  // Arm arm, Gripper gripper, DriveTrain drivetrain
  // DriveTrain drivetrain, double delay, double distance

  public ScoreMobilityDockAndEngage(Arm arm, DriveTrain drivetrain, double delay, Gripper gripper) {
    super();

    addCommands(new AutoScore(arm, drivetrain, 0, gripper));
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(new ArmDown(arm, gripper));
    addCommands(new DockAndEngage(drivetrain, 0, -2));
    addCommands(drivetrain.stop());
  }
}
