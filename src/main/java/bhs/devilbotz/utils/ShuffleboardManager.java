package bhs.devilbotz.utils;

import bhs.devilbotz.lib.AutonomousModes;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardManager {
  public static SendableChooser<AutonomousModes> autoModeChooser = new SendableChooser<>();
  private static final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

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

    addDefaultWidgets();

    initAutoModeChooser();
  }

  private void addDefaultWidgets() {
    driveTab
        .add("Auto Mode", autoModeChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
  }

  private void initAutoModeChooser() {
    autoModeChooser.setDefaultOption("Backup Short", AutonomousModes.BACKUP_SHORT);
    autoModeChooser.addOption("Backup and Balance", AutonomousModes.BACKUP_AND_BALANCE);
    autoModeChooser.addOption("Backup Far", AutonomousModes.BACKUP_FAR);
    autoModeChooser.addOption("Balance", AutonomousModes.BALANCE);
  }

  public void updateValues() {
    // TODO: add values to update on shuffleboard
  }

  // Put field
  public static void putField(Field2d field) {
    driveTab
        .add("Field", field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(2, 0)
        .withSize(5, 3);
  }
}
