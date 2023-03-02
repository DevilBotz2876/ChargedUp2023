package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToPosition extends CommandBase {
  private final Arm arm;
  private final double targetPosition;
  private final double targetPositionTolerance;

  public ArmToPosition(Arm arm, double targetPosition) {
    this.arm = arm;
    this.targetPosition = targetPosition;
    targetPositionTolerance = 5;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = arm.getPosition();

    if (currentPosition < targetPosition) {
      arm.up();
    } else if (currentPosition > targetPosition) {
      arm.down();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = arm.getPosition();
    return (currentPosition >= targetPosition)
        && (currentPosition < targetPosition + targetPositionTolerance);
  }
}
