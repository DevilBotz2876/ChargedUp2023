package bhs.devilbotz.commands;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * This is the command that is used to drive the robot. It takes in a speed and rotation value, and uses a slew rate
 * limiter to limit the acceleration of the robot.
 *
 * @since 1/18/2023
 * @author ParkerMeyers
 */
public class DriveCommand extends CommandBase {
  private final DriveTrain drive;
  private final DoubleSupplier speed;
  private final DoubleSupplier rot;

  private final SlewRateLimiter speedSlewRateLimiter =
      new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_LIMITER);

  private final SlewRateLimiter rotationSlewRateLimiter =
      new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_LIMITER);

  /**
   * The constructor for the drive command.
   * @param drive The drive train subsystem.
   * @param speed The speed of the robot.
   * @param rot The rotation of the robot.
   */
  public DriveCommand(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rot) {
    this.drive = drive;
    this.speed = speed;
    this.rot = rot;
    addRequirements(this.drive);
  }

  /**
   * Called when the command is initially scheduled.
   * @see CommandBase#initialize()
   */
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * @see CommandBase#execute()
   */
  @Override
  public void execute() {
    // The joysticks are inverted, so negate the values
    final var speed =
        -speedSlewRateLimiter.calculate(
            this.speed.getAsDouble() * Constants.DriveConstants.MAX_SPEED);

    // The rotation is inverted, so negate the value
    final var rot =
        -rotationSlewRateLimiter.calculate(
            this.rot.getAsDouble() * Constants.DriveConstants.MAX_ANGULAR_SPEED);

    drive.arcadeDrive(speed, rot);
  }

  /**
   * Called once the command ends or is interrupted.
   * @see CommandBase#end(boolean)
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns whether the command is finished.
   * @see CommandBase#isFinished()
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
