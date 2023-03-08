// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command moves the arm up.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class ArmUp extends CommandBase {
  private final Arm arm;
  private final Gripper gripper;

  /**
   * The constructor for the arm up command.
   *
   * @param arm The arm subsystem.
   */
  public ArmUp(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
    addRequirements(arm);
    addRequirements(
        gripper); // TODO: This will prevent the gripper from opening/closing while moving up
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isBottomLimit()) {
      gripper.close();
    }
    arm.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace();
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isTopLimit();
  }
}
