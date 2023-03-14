package bhs.devilbotz.commands.arm;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;

public class ArmToPosition extends ArmSafety {
  private final double targetPosition;

  public ArmToPosition(Arm arm, double targetPosition, Gripper gripper) {
    super(arm, gripper);
    this.targetPosition = targetPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initializeWithSafety() {
    CommandDebug.trace("@ position: " + getPosition() + " --> " + targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void executeWithSafety() {
    double currentPosition = getPosition();

    if (currentPosition < targetPosition) {
      up();
    } else if (currentPosition > targetPosition) {
      down();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedWithSafety() {
    double currentPosition = getPosition();

    return (currentPosition >= targetPosition)
        && (currentPosition < targetPosition + ArmConstants.POSITION_TOLERANCE);
  }
}
