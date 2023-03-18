package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class SlowRotateDriveCommand extends DriveCommand {
  public SlowRotateDriveCommand(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rot) {
    super(
        drive,
        speed,
        () -> rot.getAsDouble() / Constants.DriveConstants.PRECISION_ROTATE_SPEED_DIVIDER);
  }
}
