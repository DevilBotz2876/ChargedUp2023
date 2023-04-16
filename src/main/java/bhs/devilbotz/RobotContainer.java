// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmIdle;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmStop;
import bhs.devilbotz.commands.arm.ArmUp;
import bhs.devilbotz.commands.assist.PickupFromGround;
import bhs.devilbotz.commands.assist.PrepareForGroundPickup;
import bhs.devilbotz.commands.assist.PrepareForScore;
import bhs.devilbotz.commands.auto.AutonomousContainer;
import bhs.devilbotz.commands.drivetrain.DriveCommand;
import bhs.devilbotz.commands.drivetrain.SlowRotateDriveCommand;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperIdle;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.commands.led.SetLEDMode;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.Arduino;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.subsystems.drive.Drive;
import bhs.devilbotz.subsystems.drive.DriveIO;
import bhs.devilbotz.subsystems.drive.DriveIOCIM;
import bhs.devilbotz.subsystems.drive.DriveIOSim;
import bhs.devilbotz.utils.Alert;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Drive drive;

  private final Gripper gripper = new Gripper();

  private final Arm arm = new Arm(this);

  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager(this);

  private final Joystick leftJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT);

  private final Joystick rightJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT);

  private final Arduino arduino;

  private GamePieceTypes gamePieceType = GamePieceTypes.CONE;

  // Network Table Based Debug Status
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("Game Piece Mode");
  private StringEntry ntGamePieceMode = table.getStringTopic("state").getEntry("Cone");

  private final AutonomousContainer autonomousContainer;

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
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOCIM());
        break;

      case SIM:
        drive = new Drive(new DriveIOSim());
        break;

        // Replayed robot, disable IO implementations
      default:
        drive = new Drive(new DriveIO() {});
        break;
    }

    autonomousContainer = new AutonomousContainer(drive, arm, gripper);

    // Configure the trigger bindings
    configureBindings();

    arm.setDefaultCommand(new ArmIdle(arm));
    gripper.setDefaultCommand(new GripperIdle(gripper));

    drive.setDefaultCommand(new DriveCommand(drive, rightJoystick::getY, rightJoystick::getX));

    if (!DriverStation.isJoystickConnected(
        Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT)) {
      new Alert("Right Joystick NOT Connected!", Alert.AlertType.WARNING).set(true);
    }
    // driveTrain.setDefaultCommand(new ArcadeDriveOpenLoop(driveTrain, rightJoystick::getY,
    // rightJoystick::getX));

    buildArmShuffleboardTab();
    ntGamePieceMode.set("Cone");
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
    new JoystickButton(leftJoystick, 1)
            .onTrue(new GripperClose(gripper))
            .onFalse(new GripperIdle(gripper));

    new JoystickButton(leftJoystick, 2)
        .onTrue(new PrepareForGroundPickup(arm, gripper, this::getGamePieceType));

    new JoystickButton(leftJoystick, 3)
        .onTrue(new GripperOpen(gripper))
        .onFalse(new GripperIdle(gripper));

    new JoystickButton(leftJoystick, 4)
        .whileTrue(new ArmDown(arm, gripper))
        .onFalse(new ArmStop(arm));

    new JoystickButton(leftJoystick, 5)
        .whileTrue(new ArmUp(arm, gripper))
        .onFalse(new ArmStop(arm));

    new JoystickButton(leftJoystick, 6)
        .onTrue(
            new InstantCommand(
                () -> {
                  // set game piece to "cone"
                  this.setGamePieceType(GamePieceTypes.CONE);
                }));

    new JoystickButton(leftJoystick, 7)
        .onTrue(
            new InstantCommand(
                () -> {
                  // set game piece to "cube"
                  this.setGamePieceType(GamePieceTypes.CUBE);
                }));

    new JoystickButton(rightJoystick, 1)
        .onTrue(
            new PrepareForScore(arm, ArmConstants.POSITION_TOP, gripper, this::getGamePieceType));

    new JoystickButton(rightJoystick, 2)
        .toggleOnTrue(
            new SlowRotateDriveCommand(drive, rightJoystick::getY, rightJoystick::getX));

    new JoystickButton(rightJoystick, 3)
        .onTrue(
            new PrepareForScore(
                arm, ArmConstants.POSITION_MIDDLE, gripper, this::getGamePieceType));

    new JoystickButton(rightJoystick, 4)
        .onTrue(new ArmMoveDistance(arm, ArmConstants.POSITION_SCORING_DELTA, gripper));

    new JoystickButton(rightJoystick, 5).onTrue(new PickupFromGround(arm, gripper, drive));

    if (!DriverStation.isJoystickConnected(
        Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT)) {
      new Alert("Left Joystick NOT Connected!", Alert.AlertType.WARNING).set(true);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @param autoMode the selected autonmous mode
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutonomousModes autoMode) {
    if (autoMode == null) {
      new Alert(
              "An Autonomous mode was NOT selected. The robot will not move during autonomous",
              Alert.AlertType.ERROR)
          .set(true);
      return null;
    } else {
      return autonomousContainer.getAutonomousCommand(autoMode);
    }
  }

  /**
   * Returns the {@link ShuffleboardManager} instance.
   *
   * @return the {@link ShuffleboardManager} instance
   */
  public ShuffleboardManager getShuffleboardManager() {
    return shuffleboardManager;
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
    cmdList.add(new ArmDown(arm, gripper)).withPosition(0, 2);

    cmdList
        .add(
            "To Top",
            new PrepareForScore(arm, ArmConstants.POSITION_TOP, gripper, this::getGamePieceType))
        .withPosition(1, 0);
    cmdList
        .add(
            "To Middle",
            new PrepareForScore(arm, ArmConstants.POSITION_MIDDLE, gripper, this::getGamePieceType))
        .withPosition(1, 1);
    cmdList
        .add("To Score", new ArmMoveDistance(arm, ArmConstants.POSITION_SCORING_DELTA, gripper))
        .withPosition(1, 2);

    tab.add("Arm subsystem", arm).withPosition(0, 4);
    tab.add("Drivetrain subsystem", drive).withPosition(7, 2);
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

    cmdList
        .add(new PrepareForGroundPickup(arm, gripper, this::getGamePieceType))
        .withPosition(0, 0);
    cmdList.add(new PickupFromGround(arm, gripper, drive)).withPosition(0, 1);
    cmdList
        .add(
            "Cone Mode",
            new InstantCommand(
                () -> {
                  // set game piece to "cone"
                  this.setGamePieceType(GamePieceTypes.CONE);
                }))
        .withPosition(0, 2);
    cmdList
        .add(
            "Cube Mode",
            new InstantCommand(
                () -> {
                  // set game piece to "cube"
                  this.setGamePieceType(GamePieceTypes.CUBE);
                }))
        .withPosition(0, 3);

    CommandBase slowRotateCommand =
        new SlowRotateDriveCommand(drive, rightJoystick::getY, rightJoystick::getX);
    cmdList
        .add(
            "Normal Mode",
            new InstantCommand(
                () -> {
                  slowRotateCommand.end(true);
                }))
        .withPosition(0, 4);
    cmdList
        .add(
            "Slow Mode",
            new InstantCommand(
                () -> {
                  slowRotateCommand.schedule();
                }))
        .withPosition(0, 5);
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

  public GamePieceTypes getGamePieceType() {
    return this.gamePieceType;
  }

  public void setGamePieceType(GamePieceTypes gamePieceType) {
    this.gamePieceType = gamePieceType;
    switch (gamePieceType) {
      case CONE:
        setLEDMode(LEDModes.SET_CONE);
        ntGamePieceMode.set("Cone");
        break;
      case CUBE:
        setLEDMode(LEDModes.SET_CUBE);
        ntGamePieceMode.set("Cube");
        break;
      default:
        break;
    }
  }
}
