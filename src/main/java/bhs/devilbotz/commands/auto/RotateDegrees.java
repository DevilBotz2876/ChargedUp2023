package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Robot;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateDegrees extends CommandBase {
  private DriveTrain drivetrain;
  private PIDController rotatePid;
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
    rotatePid.enableContinuousInput(0,360);
    startAngle = drivetrain.getYaw();
    targetAngle = startAngle + degrees;
    addRequirements(drivetrain);
  }

  private void addRequirements(DriveTrain drivetrain2) {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RotateDegrees start");
    drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnError = rotatePid.calculate(drivetrain.getYaw(), targetAngle);
    drivetrain.arcadeDrive(0, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    System.out.println("RotateDegrees Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePid.atSetpoint();
  }
}
