package bhs.devilbotz.utils;

import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private static final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

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

    driveTab
        .add("Auto Mode", autoModeChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    initAutoModeChooser();
  }

  private void initAutoModeChooser() {
    autoModeChooser.setDefaultOption("Backup Short", AutonomousModes.BACKUP_SHORT);
    autoModeChooser.addOption("Backup and Balance", AutonomousModes.BACKUP_AND_BALANCE);
    autoModeChooser.addOption("Backup Far", AutonomousModes.BACKUP_FAR);
    autoModeChooser.addOption("Balance", AutonomousModes.BALANCE);
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
