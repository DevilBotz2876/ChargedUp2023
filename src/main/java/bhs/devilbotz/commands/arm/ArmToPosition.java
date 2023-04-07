package bhs.devilbotz.commands.arm;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmToPosition extends ArmSafety {
  private final double targetPosition;
  private PIDController positionPid;

  public ArmToPosition(Arm arm, double targetPosition, Gripper gripper) {
    super(arm, gripper);
    this.targetPosition = targetPosition;
    positionPid =
        new PIDController(
            ArmConstants.POSITION_P, ArmConstants.POSITION_I, ArmConstants.POSITION_D);
    positionPid.setTolerance(ArmConstants.POSITION_TOLERANCE);
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
    double speed = positionPid.calculate(currentPosition, targetPosition)/600;
    System.out.println("speed: " + speed);
    
    if (speed > 0) {
      up(speed);
    } else if (speed < 0) {
      down(speed);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedWithSafety() {
    return positionPid.atSetpoint();
  }
}
