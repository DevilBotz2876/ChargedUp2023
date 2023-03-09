// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** This command moves the arm a given distance. */
public class ArmMoveDistance extends CommandBase {
  private final Arm arm;
  private final Gripper gripper;
  private final double gripperClosePosition;
  private final double distance;
  private double start, end;

  /**
   * The constructor
   *
   * @param arm The arm subsystem.
   * @param distance Amount to move the arm.
   */
  public ArmMoveDistance(Arm arm, double distance, Gripper gripper, double gripperClosePosition) {
    this.arm = arm;
    this.distance = distance;
    this.gripper = gripper;
    this.gripperClosePosition = gripperClosePosition;

    addRequirements(arm);
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Remember our starting position.  Then apply the distance to move.  The execute method will
    // move the arm up or down
    // depending on sign of distance.  The isFinished method will check to see if current arm
    // position at end position.
    start = arm.getPosition();
    end = start + distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use sign of distance to figure out if we should move up or down.  Up is positive, down is
    // negative.
    if (distance > 0) {
      arm.up();
    } else if (distance < 0) {
      if (arm.getPosition() < gripperClosePosition) {
        gripper.close();
      }
      arm.down();
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
    double distanceSoFar = arm.getPosition() - start;
    if (distance < 0) {
      if (arm.isBottomLimit() || distanceSoFar <= distance) {
        return true;
      }
    } else if (distance > 0) {
      if (arm.isTopLimit() || distanceSoFar >= distance) {
        return true;
      }
    } else {
      return true;
    }

    return false;
  }
}
