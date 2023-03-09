package bhs.devilbotz.commands;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.utils.Alert;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * This is the command that is used to drive the robot. It takes in a speed and rotation value, and
 * uses a slew rate limiter to limit the acceleration of the robot.
 */
public class ArcadeDriveOpenLoop extends CommandBase {
  private final DriveTrain drive;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;

  private final SlewRateLimiter speedSlewRateLimiter =
      new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_LIMITER);

  private final SlewRateLimiter rotationSlewRateLimiter =
      new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_LIMITER);

  /**
   * The constructor for the drive command.
   *
   * @param drive The drive train subsystem.
   * @param speed The speed of the robot.
   * @param rotation The rotation of the robot.
   */
  public ArcadeDriveOpenLoop(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rotation) {
    this.drive = drive;
    this.speed = speed;
    this.rotation = rotation;

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
    // double s = -speed.getAsDouble();
    // double r = rotation.getAsDouble();

    double s = speedSlewRateLimiter.calculate(speed.getAsDouble());
    double r = rotationSlewRateLimiter.calculate(rotation.getAsDouble());

    drive.arcadeDriveOpenLoop(-s, r);
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
