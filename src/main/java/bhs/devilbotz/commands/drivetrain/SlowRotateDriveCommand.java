package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Constants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.drive.Drive;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleSupplier;

public class SlowRotateDriveCommand extends DriveCommand {
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("Shuffleboard/Drive/Autonomous");

  private BooleanEntry ntSlowMode = table.getBooleanTopic("Slow Mode").getEntry(false);

  public SlowRotateDriveCommand(Drive drive, DoubleSupplier speed, DoubleSupplier rot) {
    super(
        drive,
        speed,
        () -> rot.getAsDouble() / Constants.DriveConstants.PRECISION_ROTATE_SPEED_DIVIDER);
    ntSlowMode.set(false);
  }

  @Override
  public void initialize() {
    super.initialize();
    CommandDebug.trace();
    ntSlowMode.set(true);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    CommandDebug.trace();
    ntSlowMode.set(false);
  }
}
