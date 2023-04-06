// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;

/**
 * This command moves the arm down.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class ArmDown extends ArmSafety {
  private double maxSpeed;
  /**
   * The constructor for the arm down command.
   *
   * @param arm The arm subsystem.
   * @param arm The gripper subsystem.
   */
  public ArmDown(Arm arm, Gripper gripper, double maxSpeed) {
    super(arm, gripper);
    this.maxSpeed = maxSpeed;
    arm.setMaxSpeed(maxSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void executeWithSafety() {
    down();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedWithSafety() {
    return false;
  }
  ;
}
