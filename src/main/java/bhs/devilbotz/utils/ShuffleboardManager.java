package bhs.devilbotz.utils;

import bhs.devilbotz.Constants;
import bhs.devilbotz.RobotContainer;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.lib.ScoreLevels;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class manages the shuffleboard.
 *
 * @since 1/18/2023
 * @author ParkerMeyers
 */
public class ShuffleboardManager {
  /** The auto mode chooser for network tables */
  public static SendableChooser<AutonomousModes> autoModeChooser = new SendableChooser<>();

  public static SendableChooser<ScoreLevels> autoScoreLevelChooser = new SendableChooser<>();
  public static SendableChooser<GamePieceTypes> autoGamePieceTypeChooser = new SendableChooser<>();
  public static GenericEntry autoDelay;
  public static GenericEntry autoDistance;

  private static final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
  private static final ShuffleboardLayout autoMode =
      driveTab.getLayout("Autonomous", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

  private GamePieceTypes selected = GamePieceTypes.CUBE;

  private final RobotContainer robotContainer;

  private final GenericEntry gripperSetpoint;
  /** The constructor for the shuffleboard manager. */
  public ShuffleboardManager(RobotContainer robotContainer) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    this.robotContainer = robotContainer;
    int connListenerHandle =
        inst.addConnectionListener(
            true,
            event -> {
              if (event.is(NetworkTableEvent.Kind.kConnected)) {
                System.out.println("Connected to " + event.connInfo.remote_id);
              } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
                System.out.println("Disconnected from " + event.connInfo.remote_id);
              }
            });
    System.out.println(connListenerHandle);

    gripperSetpoint =
        driveTab
            .add("Gripper Setpoint", Gripper.getAtSetpoint())
            .withPosition(0, 1)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    initAutoModeChooser();
    initAutoModePreferences();
  }

  public static void putAlerts(String group, Alert.SendableAlerts sendableAlerts) {
    driveTab.add(group, sendableAlerts).withPosition(2, 2).withSize(3, 2);
  }

  private void initAutoModeChooser() {
    autoModeChooser.addOption("Sit Still", AutonomousModes.SIT_STILL);
    autoModeChooser.setDefaultOption("Mobility", AutonomousModes.MOBILITY);
    autoModeChooser.addOption("Dock/Engage (Middle)", AutonomousModes.DOCK_AND_ENGAGE);
    autoModeChooser.addOption(
        "Mobility/Dock/Engage (Wall)", AutonomousModes.MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE);
    autoModeChooser.addOption(
        "Mobility/Dock/Engage (Human)", AutonomousModes.MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE);
    autoModeChooser.addOption("Score/Mobility", AutonomousModes.SCORE_AND_MOBILITY);
    autoModeChooser.addOption("Score/Dock/Engage", AutonomousModes.SCORE_DOCK_AND_ENGAGE);
    //    autoModeChooser.addOption(
    //        "Routine 7: Score, Dock, and Engage", AutonomousModes.SCORE_DOCK_AND_ENGAGE);
    //    autoModeChooser.addOption(
    //       "Routine 8: Score, Mobility, Dock, and Engage",
    // AutonomousModes.SCORE_MOBILITY_DOCK_ENGAGE);
    //    autoModeChooser.addOption(
    //        "Routine 9: Score, Mobility, Pick, Dock, and Engage",
    //        AutonomousModes.SCORE_MOBILITY_PICK_DOCK_ENGAGE);
    //    autoModeChooser.addOption("Test", AutonomousModes.TEST);
    autoMode.add("Routine", autoModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void initAutoModePreferences() {
    autoDelay =
        autoMode.add("Delay (in seconds)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    autoDistance =
        autoMode
            .add("Distance (in meters)", Constants.DEFAULT_DISTANCE_DOCK_AND_ENGAGE)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    autoScoreLevelChooser.setDefaultOption("Low", ScoreLevels.LOW);
    autoScoreLevelChooser.addOption("Mid", ScoreLevels.MID);
    autoScoreLevelChooser.addOption("High", ScoreLevels.HIGH);
    autoMode
        .add("Score Level", autoScoreLevelChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    autoGamePieceTypeChooser.setDefaultOption("Cube", GamePieceTypes.CUBE);
    autoGamePieceTypeChooser.addOption("Cone", GamePieceTypes.CONE);
    autoGamePieceTypeChooser.addOption("OFF", GamePieceTypes.NONE);
    autoMode
        .add("Game Piece", autoGamePieceTypeChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);
  }

  /** Updates the values on the shuffleboard. */
  public void updateValues() {
    // update gripper setpoint using network tables
    gripperSetpoint.setBoolean(Gripper.getAtSetpoint());
  }

  public void updateValuesTeleop() {
    if (selected != autoGamePieceTypeChooser.getSelected()) {
      selected = autoGamePieceTypeChooser.getSelected();
      switch (selected) {
        case CONE:
          robotContainer.setLEDMode(LEDModes.SET_CONE);
          break;
        case CUBE:
          robotContainer.setLEDMode(LEDModes.SET_CUBE);
          break;
        case NONE:
          robotContainer.setLEDModeAlliance();
          break;
      }
    }
  }

  /**
   * Puts the field on the shuffleboard.
   *
   * @param field The field to put on the shuffleboard.
   */
  public static void putField(Field2d field) {
    driveTab
        .add("Field", field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(2, 0)
        .withSize(3, 2);
  }

  public static void putCamera(VideoSource videoSource) {
    driveTab.add("Camera", videoSource).withPosition(5, 0).withSize(4, 4);
  }
}
