package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAndMobility extends SequentialCommandGroup {
  public ScoreAndMobility(Arm arm, DriveTrain drivetrain, double distance, Gripper gripper) {
    super();

    addCommands(CommandDebug.start());
    addCommands(new AutoScore(arm, drivetrain, 0, gripper));
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(new ArmDown(arm, gripper));
    addCommands(new DriveStraightPID(drivetrain, distance));
    addCommands(CommandDebug.end());
  }
}
