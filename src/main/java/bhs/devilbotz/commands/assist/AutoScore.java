package bhs.devilbotz.commands.assist;

import bhs.devilbotz.commands.auto.DriveStraightPID;
import bhs.devilbotz.commands.auto.RotateDegrees;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScore extends SequentialCommandGroup {
  private final double driveBackDistance = -0.5; // in meters

  public AutoScore(Arm arm, Gripper gripper, DriveTrain drivetrain) {
    super();
    addCommands(new GripperOpen(gripper));
    addCommands(new DriveStraightPID(drivetrain, driveBackDistance));
    addCommands(new InstantCommand(() -> {
        drivetrain.tankDriveVolts(0, 0);
      }));
    addCommands(new PrepareForGroundPickup(arm, gripper));
    addCommands(new RotateDegrees(drivetrain, 180));
  }
}
