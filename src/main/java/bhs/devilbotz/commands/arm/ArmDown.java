// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command moves the arm down.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class ArmDown extends CommandBase {
  private final Arm arm;
  private final Gripper gripper;
  private final double gripperClosePosition;

  /**
   * The constructor for the arm down command.
   *
   * @param arm The arm subsystem.
   */
  public ArmDown(Arm arm, Gripper gripper, double gripperClosePosition) {
    this.arm = arm;
    this.gripper = gripper;
    this.gripperClosePosition = gripperClosePosition;

    addRequirements(arm);
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getPosition() < gripperClosePosition) {
      gripper.close();
    }
    arm.down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isBottomLimit();
  }
}
