package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateDegrees extends CommandBase {
  private DriveTrain drivetrain;
  private PIDController rotatePid;
  private double rotationAmount;
  private double startAngle;
  private double targetAngle;

  public RotateDegrees(DriveTrain drivetrain, int degrees) {
    this.drivetrain = drivetrain;
    rotatePid =
        new PIDController(
            Robot.getDriveTrainConstant("ROTATE_P").asDouble(),
            Robot.getDriveTrainConstant("ROTATE_I").asDouble(),
            Robot.getDriveTrainConstant("ROTATE_D").asDouble());
    rotatePid.setTolerance(Robot.getDriveTrainConstant("ROTATE_PID_TOLERANCE").asDouble());
    rotatePid.enableContinuousInput(0, 360);
    this.rotationAmount = degrees;
    addRequirements(drivetrain);
  }

  private void addRequirements(DriveTrain drivetrain2) {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace();
    startAngle = drivetrain.getYaw();
    targetAngle = startAngle + rotationAmount;
    CommandDebug.trace("startAngle: " + startAngle + " --> " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = drivetrain.getYaw();
    double turnError = rotatePid.calculate(currentAngle, targetAngle);
    drivetrain.arcadeDrive(0, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double currentAngle = drivetrain.getYaw();
    CommandDebug.trace("endAngle: " + currentAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePid.atSetpoint();
  }
}
