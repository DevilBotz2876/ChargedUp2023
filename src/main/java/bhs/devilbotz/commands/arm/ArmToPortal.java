package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToPortal extends CommandBase {
  private final Arm arm;

  public ArmToPortal(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.abovePortal()) {
      arm.down();

      if (arm.isBottomLimit()) {
        arm.up();
      }
    } else if (arm.belowPortal()) {
      arm.up();

      if (arm.isTopLimit()) {
        arm.down();
      }
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
    // Stop command in case the end position is out of range or encoder is broken or giving
    // incorrect readings
    return arm.atPortal();
  }
}
