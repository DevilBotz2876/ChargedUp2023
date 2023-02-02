package bhs.devilbotz.commands;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * This is the command that is used to drive the robot. It takes in a speed and rotation value, and
 * uses a slew rate limiter to limit the acceleration of the robot.
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
   *
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
    // Clamp the values to the range [0.95, 0.95] to allow the PID loop some room to work
    // TODO: replace clamp with dynamic range
    double speed = MathUtil.clamp(this.speed.getAsDouble(), -0.95, 0.95);
    double rot = MathUtil.clamp(this.rot.getAsDouble(), -0.95, 0.95);

    // Add a deadband to the joystick values
    speed = MathUtil.applyDeadband(speed, Constants.DriveConstants.JOYSTICK_DEADBAND);
    rot = MathUtil.applyDeadband(rot, Constants.DriveConstants.JOYSTICK_DEADBAND);

    // (a*(x^{3})+(b-a)*x)*c
    double a = 0.7;
    double b = 0.9091;
    double c = 1.1;

    speed = (a * (speed * speed * speed) + (b - a) * speed) * c;
    rot = (a * (rot * rot * rot) + (b - a) * rot) * c;

    // The joysticks are inverted, so negate the values
    final var calculatedSpeed =
        -speedSlewRateLimiter.calculate(speed * Constants.DriveConstants.MAX_SPEED);

    // The rotation is inverted, so negate the value
    final var calculatedRot =
        -rotationSlewRateLimiter.calculate(rot * Constants.DriveConstants.MAX_ANGULAR_SPEED);

    drive.arcadeDrive(calculatedSpeed, calculatedRot);
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
