package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateDegrees extends CommandBase {
  private Drive drive;
  private PIDController rotatePid;
  private double rotationAmount;
  private double startAngle;
  private double targetAngle;

  public RotateDegrees(Drive drive, int degrees) {
    this.drive = drive;
    rotatePid =
        new PIDController(
            Robot.getDriveConstant("ROTATE_P").asDouble(),
            Robot.getDriveConstant("ROTATE_I").asDouble(),
            Robot.getDriveConstant("ROTATE_D").asDouble());
    rotatePid.setTolerance(Robot.getDriveConstant("ROTATE_PID_TOLERANCE").asDouble());
    rotatePid.enableContinuousInput(0, 360);
    this.rotationAmount = degrees;
    addRequirements(drive);
  }

  private void addRequirements(Drive drive) {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace();
    startAngle = drive.getYaw();
    targetAngle = startAngle + rotationAmount;
    CommandDebug.trace("startAngle: " + startAngle + " --> " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = drive.getYaw();
    double turnError = rotatePid.calculate(currentAngle, targetAngle);
    drive.arcadeDrive(0, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double currentAngle = drive.getYaw();
    CommandDebug.trace("endAngle: " + currentAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePid.atSetpoint();
  }
}
