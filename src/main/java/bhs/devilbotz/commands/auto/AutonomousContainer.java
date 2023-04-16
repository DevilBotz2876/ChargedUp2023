package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.CommunityLocation;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.subsystems.drive.Drive;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** This class is used to encapsulate all of the autonmous routines */
public class AutonomousContainer {
  private final Drive drive;
  private final Arm arm;
  private final Gripper gripper;

  public AutonomousContainer(Drive drive, Arm arm, Gripper gripper) {
    this.drive = drive;
    this.arm = arm;
    this.gripper = gripper;
  }

  public Command getAutonomousCommand(AutonomousModes autoMode) {
    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup();
    double delay = ShuffleboardManager.autoDelay.getDouble(0);
    double maxDistance =
        ShuffleboardManager.autoDistance.getDouble(Constants.DEFAULT_DISTANCE_MOBILITY);
    double startAngle = drive.getYaw();

    autonomousCommand.addCommands(CommandDebug.message("Autonomous: Start"));
    // Always start autonomous with the specified delay to allow alliance members to do whatever
    // they need to do before we start
    autonomousCommand.addCommands(Commands.waitSeconds(delay));

    switch (autoMode) {
      case SIT_STILL:
        break;
      case MOBILITY:
        autonomousCommand.addCommands(new Mobility(drive, maxDistance));
        break;
      case SCORE_AND_MOBILITY:
        autonomousCommand.addCommands(new ScoreAndMobility(arm, drive, gripper));
        break;
      case DOCK_AND_ENGAGE:
        autonomousCommand.addCommands(new DockAndEngage(drive, maxDistance, startAngle));
        break;
      case MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE:
        autonomousCommand.addCommands(
            new MobilityDockAndEngage(
                drive, CommunityLocation.HUMAN, DriverStation.getAlliance(), startAngle));
        break;
      case MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE:
        autonomousCommand.addCommands(
            new MobilityDockAndEngage(
                drive, CommunityLocation.WALL, DriverStation.getAlliance(), startAngle));
        break;
      case SCORE_DOCK_AND_ENGAGE:
        autonomousCommand.addCommands(
            new ScoreDockAndEngage(arm, drive, maxDistance, gripper, startAngle));
        break;
      case SCORE_MOBILITY_DOCK_ENGAGE:
        break;
      case SCORE_MOBILITY_PICK_DOCK_ENGAGE:
        break;
      case TEST:
        break;
      default:
        break;
    }

    // Always stop the drivetrain at the end of autonomous
    autonomousCommand.addCommands(drive.stopCommand());
    autonomousCommand.addCommands(CommandDebug.message("Autonomous: End"));

    return autonomousCommand;
  }
}
