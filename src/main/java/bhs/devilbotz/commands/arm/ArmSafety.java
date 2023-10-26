package bhs.devilbotz.commands.arm;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
 *   <li>If the arm is expected to be moving (up <b>or</b> down):
 *       <ul>
 *         <li>The arm will stop automatically if the arm isn't moving (e.g. is stuck)
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
  private Timer timer = new Timer();
  private final double maxSpeed;
  private double currentSpeed;
  private double previousSpeed;
  private double speed;

  private static enum ArmCommand {
    UNKNOWN,
    STOP,
    MOVE_UP,
    MOVE_DOWN,
    EMERGENCY_STOP
  }

  public ArmSafety(Arm arm, Gripper gripper) {
    this(arm, gripper, 1.0);
  }

  public ArmSafety(Arm arm, Gripper gripper, double maxSpeed) {
    this.arm = arm;
    this.gripper = gripper;
    this.maxSpeed = Math.abs(maxSpeed);

    addRequirements(arm);
    addRequirements(gripper);
  }

  /** Command to request moving the arm down */
  public final void down() {
    this.down(1.0);
  }

  /** Command to request moving the arm up */
  public final void up() {
    this.up(1.0);
  }

  /** Command to request moving the arm down */
  public final void down(double speed) {
    currentCommand = ArmCommand.MOVE_DOWN;
    this.speed = Math.abs(speed);
  }

  /** Command to request moving the arm up */
  public final void up(double speed) {
    currentCommand = ArmCommand.MOVE_UP;
    this.speed = Math.abs(speed);
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
    double currentPosition = getPosition();
    CommandDebug.println(getClass().getName() + ":initialize @ position: " + currentPosition);
    initializeWithSafety();

    currentCommand = ArmCommand.UNKNOWN;
    previousCommand = ArmCommand.UNKNOWN;

    timer.stop();
    timer.reset();
  }

  @Override
  public final void execute() {

    double currentPosition = getPosition();
    executeWithSafety();

    if (isArmStuck()) {
      currentCommand = ArmCommand.EMERGENCY_STOP;
      CommandDebug.trace(
          "Detected stuck arm @ position:"
              + currentPosition
              + " and velocity:"
              + arm.getVelocity());
    }

    switch (currentCommand) {
      case MOVE_UP:
        //        currentSpeed = ArmConstants.SPEED_UP_MAX;
        currentSpeed = Math.min(this.speed, curve(getPosition()));
        if (arm.isTopLimit()) {
          // Prevent the arm from moving up if at the top limit
          CommandDebug.trace("Top Limit Reached @ position: " + currentPosition);
          currentCommand = ArmCommand.EMERGENCY_STOP;
        } else if (arm.isBottomLimit()) {
          // We want to close the gripper if we are moving up but at the bottom
          CommandDebug.trace("Auto Closing Gripper bottom limit @ position: " + currentPosition);
          gripper.close();
        }
        break;
      case MOVE_DOWN:
        //        currentSpeed = ArmConstants.SPEED_DOWN_MAX;
        currentSpeed = Math.min(this.speed, curve(getPosition()));
        if (arm.isBottomLimit()) {
          // Prevent the arm from moving down if at the top limit
          CommandDebug.trace("Bottom Limit Reached @ position: " + currentPosition);
          currentCommand = ArmCommand.EMERGENCY_STOP;
        } else {
          if (currentPosition < ArmConstants.POSITION_GRIPPER_CLOSE) {
            // We want to close the gripper if we are moving down and nearing the bottom
            CommandDebug.trace("Auto Closing Gripper @ position: " + currentPosition);
            gripper.close();
          }
        }
        break;
      default:
        break;
    }

    // Make sure we don't exceed the max speed requested
    currentSpeed = MathUtil.clamp(currentSpeed, 0, maxSpeed);

    // Send the actual request to the arm subsystem
    if ((currentCommand != previousCommand) || (currentSpeed != previousSpeed)) {
      switch (currentCommand) {
        case MOVE_UP:
          CommandDebug.println(
              getClass().getName()
                  + ":move up @ position: "
                  + currentPosition
                  + " @ speed: "
                  + currentSpeed);
          arm.up(currentSpeed);
          break;

        case MOVE_DOWN:
          CommandDebug.println(
              getClass().getName()
                  + ":move down @ position: "
                  + currentPosition
                  + " @ speed: "
                  + currentSpeed);
          arm.down(currentSpeed);
          break;

        case STOP:
        case EMERGENCY_STOP:
        default:
          CommandDebug.println(getClass().getName() + ":stop @ position: " + currentPosition);
          arm.stop();
          break;
      }

      previousCommand = currentCommand;
      previousSpeed = currentSpeed;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public final void end(boolean interrupted) {

    double currentPosition = getPosition();
    CommandDebug.println(
        getClass().getName()
            + ":end @ position: "
            + currentPosition
            + " Interrupted? "
            + interrupted);
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

  private boolean isArmStuck() {
    if (Robot.isSimulation()
        || (currentCommand.equals(ArmCommand.MOVE_UP) && !DriverStation.isAutonomous())) {
      return false;
    }
    // check the rate of change of the arm position
    // if there is no change, then the rope is broken

    double currentSpeed = Math.abs(arm.getVelocity());

    // start a timer to see if the rope is stuck for a while
    // if it is, then we need to stop the arm
    if (currentSpeed < ArmConstants.SPEED_TO_DECIDE_ARM_STUCK) {
      timer.start();

    } else {
      timer.stop();
      timer.reset();
    }

    if (DriverStation.isAutonomous()) {
      return timer.hasElapsed(1);
    } else {
      return timer.hasElapsed(ArmConstants.DURATION_TO_DECIDE_ARM_STUCK);
    }
  }

  public static double curve(double x) {
    if (x < 0) {
      x = 0;
    } else if (x > 600) {
      x = 600;
    }
    if (x <= 300) {
      return 0.575 + (x / 300) * 0.65;
    } else {
      return 0.95 - ((x - 300) / 300) * 0.65;
    }
  }
}
