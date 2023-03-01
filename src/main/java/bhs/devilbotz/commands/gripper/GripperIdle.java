// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.gripper;

import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command stops the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class GripperIdle extends CommandBase {
  private final Gripper gripper;

  /**
   * The constructor for the gripper idle command.
   *
   * @param gripper The gripper subsystem.
   */
  public GripperIdle(Gripper gripper) {
    this.gripper = gripper;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
