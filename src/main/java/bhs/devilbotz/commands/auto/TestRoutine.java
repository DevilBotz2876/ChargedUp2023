package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmUp;
import bhs.devilbotz.commands.drivetrain.ArcadeDriveOpenLoop;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.commands.led.SetLEDMode;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.Arduino;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestRoutine extends SequentialCommandGroup {
  public TestRoutine(Arm arm, Gripper gripper, Arduino arduino, DriveTrain drive) {
    super();

    addCommands(CommandDebug.start());

    // Test LEDs
    addCommands(new SetLEDMode(arduino, LEDModes.SET_RED));
    addCommands(new WaitCommand(0.5));
    addCommands(new SetLEDMode(arduino, LEDModes.SET_BLUE));
    addCommands(new WaitCommand(0.5));
    addCommands(new SetLEDMode(arduino, LEDModes.SET_CONE));
    addCommands(new WaitCommand(0.5));
    addCommands(new SetLEDMode(arduino, LEDModes.SET_CUBE));
    addCommands(new WaitCommand(0.5));

    // Drive Forward 2m
    addCommands(new DriveStraightPID(drive, 1));
    addCommands(new WaitCommand(1));
    // Drive Backwards 2m
    addCommands(new DriveStraightPID(drive, -1));
    addCommands(new WaitCommand(1));
    // Turn Left and exit the command after 1 second
    addCommands(
        new ParallelRaceGroup(
            new ArcadeDriveOpenLoop(drive, () -> 0, () -> 0.5), new WaitCommand(1)));
    addCommands(new WaitCommand(1));
    // Turn Right 90
    addCommands(
        new ParallelRaceGroup(
            new ArcadeDriveOpenLoop(drive, () -> 0, () -> -0.5), new WaitCommand(1)));
    addCommands(new WaitCommand(1));

    if (!arm.isBottomLimit()) {
      addCommands(new ArmDown(arm, gripper));
      addCommands(new WaitCommand(1));
    }
    // move arm to the top
    addCommands(new ArmUp(arm, gripper));
    addCommands(new WaitCommand(1));

    // Repeat gripper twice
    for (int i = 0; i < 2; i++) {
      addCommands(new GripperOpen(gripper));
      addCommands(new WaitCommand(1));
      addCommands(new GripperClose(gripper));
      addCommands(new WaitCommand(1));
    }

    // move arm to the bottom
    addCommands(new ArmDown(arm, gripper));
    addCommands(new WaitCommand(1));

    addCommands(CommandDebug.end());
  }
}
