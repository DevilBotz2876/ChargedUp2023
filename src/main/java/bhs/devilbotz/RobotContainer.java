// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.commands.BalancePID;
import bhs.devilbotz.commands.DriveCommand;
import bhs.devilbotz.commands.DriveStraightPID;
import bhs.devilbotz.commands.DriveStraightToDock;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmStop;
import bhs.devilbotz.commands.arm.ArmUp;
import bhs.devilbotz.commands.auto.BalanceAuto;
import bhs.devilbotz.commands.auto.TestAuto;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperIdle;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.RobotConfig;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final HashMap<AutonomousModes, Command> autoCommands = new HashMap<>();

  private DriveTrain driveTrain = null;
  private Gripper gripper = null;

  private final Arm arm = new Arm();

  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

  private final Joystick joystick =
      new Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    if (RobotConfig.isCompBot()) {
      gripper = new Gripper();
    }

    // Configure the trigger bindings
    configureBindings();
    buildAutoCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, joystick::getY, joystick::getX));

    // For testing
    new JoystickButton(joystick, 1).toggleOnTrue(new BalancePID(driveTrain));

    new JoystickButton(joystick, 6).whileTrue(new ArmUp(arm)).onFalse(new ArmStop(arm));
    new JoystickButton(joystick, 7).whileTrue(new ArmDown(arm)).onFalse(new ArmStop(arm));

    new JoystickButton(joystick, 4)
        .toggleOnTrue(new GripperOpen(gripper))
        .onFalse(new GripperIdle(gripper));
    new JoystickButton(joystick, 5)
        .toggleOnTrue(new GripperClose(gripper))
        .onFalse(new GripperIdle(gripper));
  }

  private void buildAutoCommands() {
    autoCommands.put(AutonomousModes.BALANCE, new BalanceAuto(driveTrain));
    autoCommands.put(
        AutonomousModes.DRIVE_STRAIGHT_DISTANCE_PID, new DriveStraightPID(driveTrain, 10));

    autoCommands.put(AutonomousModes.TEST, new TestAuto(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @param autoMode the selected autonmous mode
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutonomousModes autoMode) {
    Command autonomousCommand = null;

    if (autoMode == null) {
      System.out.println("Robot will NOT move during autonomous :/// You screwed something up");
    } else {
      switch (autoMode) {
        case SIT_STILL:
          break;
        case MOBILITY:
          autonomousCommand =
              new DriveStraightPID(
                  driveTrain,
                  ShuffleboardManager.autoDistance.getDouble(Constants.DEFAULT_DISTANCE_MOBILITY));
          break;
        case SCORE_AND_MOBILITY:
          break;
        case DOCK_AND_ENGAGE:
          autonomousCommand =
              new DriveStraightToDock(
                      driveTrain,
                      ShuffleboardManager.autoDistance.getDouble(
                          Constants.DEFAULT_DISTANCE_DOCK_AND_ENGAGE))
                  .andThen(new BalancePID(driveTrain));
          new BalancePID(driveTrain);
          break;
        case MOBILITY_DOCK_AND_ENGAGE:
          break;

        case SCORE_DOCK_AND_ENGAGE:
          break;

        case SCORE_MOBILITY_DOCK_ENGAGE:
          break;
        case SCORE_MOBILITY_PICK_DOCK_ENGAGE:
          break;
        default:
          autonomousCommand = autoCommands.get(autoMode);
      }
    }

    return autonomousCommand;
  }

  /**
   * Returns the {@link ShuffleboardManager} instance.
   *
   * @return the {@link ShuffleboardManager} instance
   */
  public ShuffleboardManager getShuffleboardManager() {
    return shuffleboardManager;
  }

  /**
   * Resets the robots position to the pose
   *
   * @see DriveTrain#resetRobotPosition()
   */
  public void resetRobotPosition() {
    driveTrain.resetRobotPosition();
  }

  public void enableCompressor() {
    if (gripper != null) {
      gripper.enableCompressor();
    }
  }
}
