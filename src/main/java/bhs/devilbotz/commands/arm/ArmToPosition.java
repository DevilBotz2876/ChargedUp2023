package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToPosition extends CommandBase {
  private final Arm arm;
  private final double targetPosition;
  private final double targetPositionTolerance;

  private Gripper gripper;
  private double gripperClosePosition;
  private boolean bReachedLimit;

  public ArmToPosition(Arm arm, double targetPosition) {
    this.arm = arm;
    this.targetPosition = targetPosition;
    targetPositionTolerance = 5;
    bReachedLimit = false;
    gripper = null;
    gripperClosePosition = 0;
    addRequirements(arm);
  }

  public ArmToPosition(
      Arm arm, double targetPosition, Gripper gripper, double gripperClosePosition) {
    this(arm, targetPosition);
    this.gripper = gripper;
    this.gripperClosePosition = gripperClosePosition;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = arm.getPosition();

    if (currentPosition < targetPosition) {
      if (arm.isTopLimit()) {
        bReachedLimit = true;
      } else {
        arm.up();
      }
    } else if (currentPosition > targetPosition) {
      if (arm.isBottomLimit()) {
        bReachedLimit = true;
      } else {
        arm.down();
        if ((null != gripper) && (currentPosition < gripperClosePosition)) {
          gripper.close();
        }
      }
    }

    if (bReachedLimit) {
      arm.stop();
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

    if (bReachedLimit) {
      return true;
    }
    return (currentPosition >= targetPosition)
        && (currentPosition < targetPosition + targetPositionTolerance);
  }
}
