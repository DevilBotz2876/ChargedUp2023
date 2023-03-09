// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.Gripper;

/** This command moves the arm a given distance. */
public class ArmMoveDistance extends ArmSafety {
  private final double distance;
  private double start;

  /**
   * The constructor
   *
   * @param arm The arm subsystem.
   * @param distance Amount to move the arm.
   */
  public ArmMoveDistance(Arm arm, double distance, Gripper gripper) {
    super(arm, gripper);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initializeWithSafety() {
    // Remember our starting position.  Then apply the distance to move.  The execute method will
    // move the arm up or down
    // depending on sign of distance.  The isFinished method will check to see if current arm
    // position at end position.
    start = getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void executeWithSafety() {
    // Use sign of distance to figure out if we should move up or down.  Up is positive, down is
    // negative.
    if (distance > 0) {
      up();
    } else if (distance < 0) {
      down();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedWithSafety() {
    // Stop command in case the end position is out of range or encoder is broken or giving
    // incorrect readings
    double distanceSoFar = getPosition() - start;
    return (Math.abs(distanceSoFar) >= Math.abs(distance));
  }
}
