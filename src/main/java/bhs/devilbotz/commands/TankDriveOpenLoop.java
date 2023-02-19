package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
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
   * @param speed The speed of the robot.
   * @param rot The rotation of the robot.
   */
  public TankDriveOpenLoop(DriveTrain drive, DoubleSupplier left, DoubleSupplier right) {
    this.drive = drive;
    this.left = left;
    this.right = right;

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
