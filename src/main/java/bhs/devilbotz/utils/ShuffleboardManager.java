package bhs.devilbotz.utils;

import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.lib.ScoreLevels;
import bhs.devilbotz.subsystems.Gripper;
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

  private final GenericEntry gripperSetpoint;
  /** The constructor for the shuffleboard manager. */
  public ShuffleboardManager() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
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

  private void initAutoModeChooser() {
    autoModeChooser.addOption("Routine 1: Sit Still", AutonomousModes.SIT_STILL);
    autoModeChooser.setDefaultOption("Routine 2: Mobility", AutonomousModes.MOBILITY);
    autoModeChooser.addOption("Routine 3: Score and Mobility", AutonomousModes.SCORE_AND_MOBILITY);
    autoModeChooser.addOption("Routine 4: Dock and Engage", AutonomousModes.DOCK_AND_ENGAGE);
    autoModeChooser.addOption(
        "Routine 5: Mobility, Dock, and Engage", AutonomousModes.MOBILITY_DOCK_AND_ENGAGE);
    autoModeChooser.addOption(
        "Routine 6: Score, Dock, and Engage", AutonomousModes.SCORE_DOCK_AND_ENGAGE);
    autoModeChooser.addOption(
        "Routine 7: Score, Mobility, Dock, and Engage", AutonomousModes.SCORE_MOBILITY_DOCK_ENGAGE);
    autoModeChooser.addOption(
        "Routine 8: Score, Mobility, Pick, Dock, and Engage",
        AutonomousModes.SCORE_MOBILITY_PICK_DOCK_ENGAGE);

    autoModeChooser.addOption("Backup Short", AutonomousModes.BACKUP_SHORT);
    autoModeChooser.addOption("Backup and Balance", AutonomousModes.BACKUP_AND_BALANCE);
    autoModeChooser.addOption("Backup Far", AutonomousModes.BACKUP_FAR);
    autoModeChooser.addOption("Balance", AutonomousModes.BALANCE);
    autoModeChooser.addOption("Drive Distance", AutonomousModes.DRIVE_DISTANCE);
    autoModeChooser.addOption("Drive Distance PID", AutonomousModes.DRIVE_DISTANCE_PID);
    autoMode.add("Routine", autoModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void initAutoModePreferences() {
    autoDelay =
        autoMode.add("Delay (in seconds)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    autoDistance =
        autoMode.add("Distance (in meters)", 5).withWidget(BuiltInWidgets.kTextView).getEntry();

    autoScoreLevelChooser.setDefaultOption("Low", ScoreLevels.LOW);
    autoScoreLevelChooser.addOption("Mid", ScoreLevels.MID);
    autoScoreLevelChooser.addOption("High", ScoreLevels.HIGH);
    autoMode
        .add("Score Level", autoScoreLevelChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    autoGamePieceTypeChooser.setDefaultOption("Cube", GamePieceTypes.CUBE);
    autoGamePieceTypeChooser.addOption("Cone", GamePieceTypes.CONE);
    autoMode
        .add("Game Piece", autoGamePieceTypeChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);
  }

  /** Updates the values on the shuffleboard. */
  public void updateValues() {
    // update gripper setpoint using network tables
    gripperSetpoint.setBoolean(Gripper.getAtSetpoint());
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
        .withSize(5, 3);
  }
}
