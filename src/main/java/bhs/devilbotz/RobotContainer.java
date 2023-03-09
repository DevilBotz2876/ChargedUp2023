// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.DriveCommand;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmIdle;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmStop;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.commands.arm.ArmUp;
import bhs.devilbotz.commands.assist.PickupFromGround;
import bhs.devilbotz.commands.assist.PrepareForGroundPickup;
import bhs.devilbotz.commands.auto.DockAndEngage;
import bhs.devilbotz.commands.auto.Mobility;
import bhs.devilbotz.commands.auto.MobilityDockAndEngage;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperIdle;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.commands.led.SetLEDMode;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.CommunityLocation;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.Arduino;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.Alert;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();

  private final Gripper gripper = new Gripper();

  private final Arm arm = new Arm(this);

  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager(this);

  private final Joystick leftJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT);

  private final Joystick rightJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT);

  private final Arduino arduino;

  {
    try {
      arduino = new Arduino();
    } catch (Exception e) {
      new Alert("Arduino is disconnected. LEDs will not function", Alert.AlertType.ERROR).set(true);
      throw new RuntimeException(e);
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData(driveTrain);

    arm.setDefaultCommand(new ArmIdle(arm));
    gripper.setDefaultCommand(new GripperIdle(gripper));

    if (Robot.isReal()
        || DriverStation.isJoystickConnected(
            Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT)) {
      driveTrain.setDefaultCommand(
          new DriveCommand(driveTrain, rightJoystick::getY, rightJoystick::getX));
    } else {
      driveTrain.setDefaultCommand(driveTrain.stop());
    }
    if (false
        == DriverStation.isJoystickConnected(
            Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT)) {
      new Alert("Right Joystick NOT Connected!", Alert.AlertType.WARNING).set(true);
    }
    // driveTrain.setDefaultCommand(new ArcadeDriveOpenLoop(driveTrain, rightJoystick::getY,
    // rightJoystick::getX));

    buildArmShuffleboardTab();
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
    if (Robot.isReal()
        || DriverStation.isJoystickConnected(
            Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT)) {
      new JoystickButton(leftJoystick, 1)
          .onTrue(new GripperClose(gripper))
          .onFalse(new GripperIdle(gripper));

      new JoystickButton(leftJoystick, 2).onTrue(new PrepareForGroundPickup(arm, gripper));

      new JoystickButton(leftJoystick, 3)
          .onTrue(new GripperOpen(gripper))
          .onFalse(new GripperIdle(gripper));

      new JoystickButton(leftJoystick, 4)
          .whileTrue(new ArmDown(arm, gripper, ArmConstants.POSITION_GRIPPER_CLOSE))
          .onFalse(new ArmStop(arm));

      new JoystickButton(leftJoystick, 5)
          .whileTrue(new ArmUp(arm, gripper))
          .onFalse(new ArmStop(arm));

      new JoystickButton(leftJoystick, 6).onTrue(new SetLEDMode(arduino, LEDModes.SET_CONE));

      new JoystickButton(leftJoystick, 7).onTrue(new SetLEDMode(arduino, LEDModes.SET_CUBE));

      new JoystickButton(rightJoystick, 1)
          .onTrue(
              new ArmToPosition(
                  arm, ArmConstants.POSITION_TOP, gripper, ArmConstants.POSITION_GRIPPER_CLOSE));

      new JoystickButton(rightJoystick, 3)
          .onTrue(
              new ArmToPosition(
                  arm, ArmConstants.POSITION_MIDDLE, gripper, ArmConstants.POSITION_GRIPPER_CLOSE));

      new JoystickButton(rightJoystick, 4)
          .onTrue(
              new ArmMoveDistance(
                  arm,
                  ArmConstants.POSITION_SCORING_DELTA,
                  gripper,
                  ArmConstants.POSITION_GRIPPER_CLOSE));
    }
    if (false
        == DriverStation.isJoystickConnected(
            Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT)) {
      new Alert("Left Joystick NOT Connected!", Alert.AlertType.WARNING).set(true);
    }

    SmartDashboard.putData("gripperOpen", new GripperOpen(gripper));
    SmartDashboard.putData("gripperClose", new GripperClose(gripper));

    SmartDashboard.putData(
        "armScorePiece",
        new ArmMoveDistance(
                arm,
                ArmConstants.POSITION_SCORING_DELTA,
                gripper,
                ArmConstants.POSITION_GRIPPER_CLOSE)
            .andThen(new GripperOpen(gripper)));

    /*
    new JoystickButton(leftJoystick, 6)
            .whileTrue( Cone Mode );

    new JoystickButton(leftJoystick, 7)
            .whileTrue( Cube Mode );
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @param autoMode the selected autonmous mode
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutonomousModes autoMode) {
    Command autonomousCommand = null;
    double delay = ShuffleboardManager.autoDelay.getDouble(0);
    double maxDistance =
        ShuffleboardManager.autoDistance.getDouble(Constants.DEFAULT_DISTANCE_MOBILITY);

    if (autoMode == null) {
      new Alert(
              "An Autonomous mode was NOT selected. The robot will not move during autonomous",
              Alert.AlertType.ERROR)
          .set(true);
    } else {
      switch (autoMode) {
        case SIT_STILL:
          break;
        case MOBILITY:
          autonomousCommand = new Mobility(driveTrain, delay, maxDistance);
          break;
        case SCORE_AND_MOBILITY:
          break;
        case DOCK_AND_ENGAGE:
          autonomousCommand = new DockAndEngage(driveTrain, delay, maxDistance);
          break;
        case MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE:
          autonomousCommand =
              new MobilityDockAndEngage(
                  driveTrain, delay, CommunityLocation.HUMAN, DriverStation.getAlliance());

        case MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE:
          autonomousCommand =
              new MobilityDockAndEngage(
                  driveTrain, delay, CommunityLocation.WALL, DriverStation.getAlliance());
          break;
        case SCORE_DOCK_AND_ENGAGE:
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

  /** Initialize gripper to known/same position */
  public void initGripper() {
    gripper.close();
  }

  private void buildArmShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    ShuffleboardContainer cmdList =
        tab.getLayout("Arm Commands", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(4, 4)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

    cmdList.add(new ArmStop(arm)).withPosition(0, 0);
    cmdList.add(new ArmUp(arm, gripper)).withPosition(0, 1);
    cmdList.add(new ArmDown(arm, gripper, ArmConstants.POSITION_GRIPPER_CLOSE)).withPosition(0, 2);

    cmdList
        .add(
            "To Top",
            new ArmToPosition(
                arm, ArmConstants.POSITION_TOP, gripper, ArmConstants.POSITION_GRIPPER_CLOSE))
        .withPosition(1, 0);
    cmdList
        .add(
            "To Middle",
            new ArmToPosition(
                arm, ArmConstants.POSITION_MIDDLE, gripper, ArmConstants.POSITION_GRIPPER_CLOSE))
        .withPosition(1, 1);
    cmdList
        .add(
            "To Bottom",
            new ArmToPosition(
                arm, ArmConstants.POSITION_BOTTOM, gripper, ArmConstants.POSITION_GRIPPER_CLOSE))
        .withPosition(1, 2);
    cmdList
        .add(
            "To Score",
            new ArmMoveDistance(
                arm,
                ArmConstants.POSITION_SCORING_DELTA,
                gripper,
                ArmConstants.POSITION_GRIPPER_CLOSE))
        .withPosition(1, 2);

    tab.add("Arm subsystem", arm).withPosition(0, 4);
    tab.add("Drivetrain subsystem", arm).withPosition(7, 2);
    buildGripperShuffleboardTab();
    buildDriverAssistShuffleboardTab();
  }

  private void buildGripperShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    ShuffleboardContainer cmdList =
        tab.getLayout("Grippers Commands", BuiltInLayouts.kGrid)
            .withPosition(0, 5)
            .withSize(4, 1)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

    cmdList.add(new GripperOpen(gripper)).withPosition(0, 0);
    cmdList.add(new GripperClose(gripper)).withPosition(1, 0);

    tab.add("Gripper subsystem", gripper).withPosition(7, 4);
  }

  private void buildDriverAssistShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    ShuffleboardContainer cmdList =
        tab.getLayout("Driver Assist", BuiltInLayouts.kGrid)
            .withPosition(4, 0)
            .withSize(3, 6)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 4));

    cmdList.add(new PrepareForGroundPickup(arm, gripper)).withPosition(0, 0);
    cmdList.add(new PickupFromGround(arm, gripper, driveTrain)).withPosition(0, 1);
  }

  public void setLEDModeAlliance() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      setLEDMode(LEDModes.SET_RED);
    } else {
      setLEDMode(LEDModes.SET_BLUE);
    }
  }

  public void setLEDMode(LEDModes mode) {
    new SetLEDMode(arduino, mode).schedule();
  }
}
