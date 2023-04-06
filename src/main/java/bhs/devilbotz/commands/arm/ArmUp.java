// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;

/**
 * This command moves the arm up.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class ArmUp extends ArmSafety {
  /**
   * The constructor for the arm up command.
   *
   * @param arm The arm subsystem.
   */
  public ArmUp(Arm arm, Gripper gripper) {
    super(arm, gripper);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void executeWithSafety() {
    up();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedWithSafety() {
    return false;
  }
}
