package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.CommunityLocation;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** This class is used to encapsulate all of the autonmous routines */
public class AutonomousContainer {
  private final DriveTrain drivetrain;
  private final Arm arm;
  private final Gripper gripper;

  public AutonomousContainer(DriveTrain drivetrain, Arm arm, Gripper gripper) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.gripper = gripper;
  }

  public Command getAutonomousCommand(AutonomousModes autoMode) {
    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup();
    double delay = ShuffleboardManager.autoDelay.getDouble(0);
    double maxDistance =
        ShuffleboardManager.autoDistance.getDouble(Constants.DEFAULT_DISTANCE_MOBILITY);
    double startAngle = drivetrain.getYaw();

    autonomousCommand.addCommands(CommandDebug.message("Autonomous: Start"));
    // Always start autonomous with the specified delay to allow alliance members to do whatever
    // they need to do before we start
    autonomousCommand.addCommands(Commands.waitSeconds(delay));

    switch (autoMode) {
      case SIT_STILL:
        break;
      case MOBILITY:
        autonomousCommand.addCommands(new Mobility(drivetrain, maxDistance));
        break;
      case SCORE_AND_MOBILITY:
        autonomousCommand.addCommands(new ScoreAndMobility(arm, drivetrain, maxDistance, gripper));
        break;
      case DOCK_AND_ENGAGE:
        autonomousCommand.addCommands(new DockAndEngage(drivetrain, maxDistance, startAngle));
        break;
      case MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE:
        autonomousCommand.addCommands(
            new MobilityDockAndEngage(
                drivetrain, CommunityLocation.HUMAN, DriverStation.getAlliance(), startAngle));
        break;
      case MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE:
        autonomousCommand.addCommands(
            new MobilityDockAndEngage(
                drivetrain, CommunityLocation.WALL, DriverStation.getAlliance(), startAngle));
        break;
      case SCORE_DOCK_AND_ENGAGE:
        autonomousCommand.addCommands(
            new ScoreDockAndEngage(arm, drivetrain, maxDistance, gripper, startAngle)
            // new ScoreMobilityDock(arm, drivetrain, maxDistance, gripper, startAngle)
            );
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
    autonomousCommand.addCommands(drivetrain.stopCommand());
    autonomousCommand.addCommands(CommandDebug.message("Autonomous: End"));

    return autonomousCommand;
  }
}
