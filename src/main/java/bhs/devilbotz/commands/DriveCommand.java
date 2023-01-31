package bhs.devilbotz.commands;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
  private final DriveTrain drive;
  private final DoubleSupplier speed;
  private final DoubleSupplier rot;

  private final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(10);

  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(10);

  public DriveCommand(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rot) {
    this.drive = drive;
    this.speed = speed;
    this.rot = rot;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The joysticks are inverted, so negate the values
    final var speed =
        -speedSlewRateLimiter.calculate(
            this.speed.getAsDouble() * Constants.DriveConstants.maxSpeed);

    // The rotation is inverted, so negate the value
    final var rot =
        -rotationSlewRateLimiter.calculate(
            this.rot.getAsDouble() * Constants.DriveConstants.maxAngularSpeed);

    drive.arcadeDrive(speed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
