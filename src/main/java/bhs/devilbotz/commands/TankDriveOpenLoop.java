package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.utils.Alert;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * This is the command that is used to drive the robot. It takes in a speed and rotation value, and
 * uses a slew rate limiter to limit the acceleration of the robot.
 */
public class TankDriveOpenLoop extends CommandBase {
  private final DriveTrain drive;
  private final DoubleSupplier left;
  private final DoubleSupplier right;

  /**
   * The constructor for the drive command.
   *
   * @param drive The drive train subsystem.
   * @param left The speed of the left side of the drive train.
   * @param right The speed of the right side of the drive train.
   */
  public TankDriveOpenLoop(DriveTrain drive, DoubleSupplier left, DoubleSupplier right) {
    this.drive = drive;
    this.left = left;
    this.right = right;

    new Alert("PID Driving is NOT being used", Alert.AlertType.INFO).set(true);

    addRequirements(this.drive);
  }

  /**
   * Called when the command is initially scheduled.
   *
   * @see CommandBase#initialize()
   */
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   *
   * @see CommandBase#execute()
   */
  @Override
  public void execute() {

    drive.tankDriveOpenLoop(left.getAsDouble(), right.getAsDouble());
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @see CommandBase#end(boolean)
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns whether the command is finished.
   *
   * @see CommandBase#isFinished()
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
