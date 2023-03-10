package bhs.devilbotz.commands.arm;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This is an abstract command that implements basic arm/gripper safety. The following safety checks
 * are implemented:
 *
 * <ul>
 *   <li>If the arm is moving <b>up</b>:
 *       <ul>
 *         <li>The arm will stop automatically when the <i>top/upper limit</i> switch is reached
 *         <li>If arm is at the bottom limit, the gripper will close automatically to avoid scraping
 *             against the robot frame
 *       </ul>
 *   <li>If the arm is moving <b>down</b>:
 *       <ul>
 *         <li>The arm will stop automatically when the <i>bottom/lower</i> limit switch is reached
 *         <li>If the arm is near the bottom limit, the gripper wil close automatically to avoid
 *             hitting the robot frame
 *       </ul>
 * </ul>
 *
 * <i>Note: ALL arm commands that move the arm should extend this class and must implement the
 * following two methods</i>
 */
public abstract class ArmSafety extends CommandBase {
  private final Arm arm;
  private final Gripper gripper;
  private ArmCommand currentCommand = ArmCommand.UNKNOWN;
  private ArmCommand previousCommand = ArmCommand.UNKNOWN;

  private static enum ArmCommand {
    UNKNOWN,
    STOP,
    MOVE_UP,
    MOVE_DOWN,
    EMERGENCY_STOP,
  };

  public ArmSafety(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
    currentCommand = ArmCommand.UNKNOWN;

    addRequirements(arm);
    addRequirements(gripper);
  }

  /** Command to request moving the arm down */
  public final void down() {
    currentCommand = ArmCommand.MOVE_DOWN;
  }

  /** Command to request moving the arm up */
  public final void up() {
    currentCommand = ArmCommand.MOVE_UP;
  }

  /** Command to request stopping the arm */
  public final void stop() {
    currentCommand = ArmCommand.STOP;
  }

  /** Command to return the current position of the arm */
  public final double getPosition() {
    return arm.getPosition();
  }

  @Override
  public final void initialize() {
    initializeWithSafety();

    currentCommand = ArmCommand.UNKNOWN;
    previousCommand = ArmCommand.UNKNOWN;
  }

  @Override
  public final void execute() {
    double currentPosition = getPosition();
    executeWithSafety();

    switch (currentCommand) {
      case MOVE_UP:
        if (arm.isTopLimit()) {
          // Prevent the arm from moving up if at the top limit
          currentCommand = ArmCommand.EMERGENCY_STOP;
        } else if (arm.isBottomLimit()) {
          // We want to close the gripper if we are moving up but at the bottom
          gripper.close();
        }
        break;
      case MOVE_DOWN:
        if (arm.isBottomLimit()) {
          // Prevent the arm from moving down if at the top limit
          currentCommand = ArmCommand.EMERGENCY_STOP;
        } else if (currentPosition < ArmConstants.POSITION_GRIPPER_CLOSE) {
          // We want to close the gripper if we are moving down and nearing the bottom
          gripper.close();
        }
        break;
      default:
        break;
    }

    // Send the actual request to the arm subsystem
    if (currentCommand != previousCommand) {
      switch (currentCommand) {
        case MOVE_UP:
          arm.up();
          break;

        case MOVE_DOWN:
          arm.down();
          break;

        case STOP:
        case EMERGENCY_STOP:
        default:
          arm.stop();
          break;
      }

      previousCommand = currentCommand;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public final void end(boolean interrupted) {
    // We always want to stop the arm at the end of any command
    arm.stop();
  }

  @Override
  public final boolean isFinished() {
    // The command will finish when the limits have been reached or the child says it is done
    return ((ArmCommand.EMERGENCY_STOP == currentCommand) || isFinishedWithSafety());
  }

  /**
   * The child must use this function to request arm movement using the following functions:
   *
   * <p>{@link #up()} {@link #down()} {@link #stop()} {@link #getPosition()}
   */
  public abstract void executeWithSafety();

  /** The child must use this function to indicate if the command is finished.` */
  public abstract boolean isFinishedWithSafety();

  /**
   * The child can use this function to initialize any settings immediately prior to the first
   * execution of the command
   */
  public void initializeWithSafety() {}
  ;
}
