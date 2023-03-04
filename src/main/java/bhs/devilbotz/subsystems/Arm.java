// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;

/**
 * This subsystem controls arm.
 *
 * <p>The arm must start matches fully inside the robot chassis. The arm moves up using two (gas?)
 * struts. It is lowered by a motor turning a winch. A rope winds around the winch pulling the arm
 * down. In this design it is not possible to move the arm too far up and break it. But it is
 * possible for the rope around the winch to unwind completely and start winding itself again in the
 * opposite direction. This will result in 'up' commands moving the arm down (and vice versa). It is
 * also possible for the rope to become tangled around the winch/shaft if there is enough slack or
 * extra rope.
 *
 * <p>It is possible to break the arm if it is lowered too far. The motor will be winding up the
 * rope and pulling the arm down into the chassis. The arm will hit a hard stop and the motor will
 * be pulling the arm against this hard stop. It is important therefore to detect when the arm
 * reaches top most position bottom most position. We will use limit switches to do this.
 *
 * <p>There will also be an encoder mounted on the shoulder joint. Using an encoder on the shoulder
 * joint instead of on the motor or winch shaft should provide more accurate measurement of the arm
 * position. Measuring the rotations of the winch might be less accurate because the rope is winding
 * around it in an inconsistent manner. Maybe this is neglible?
 *
 * <p>There are methods to move the arm up and down at a set speed. We do not intend to vary the
 * speed of the arm or expose speed control outside the arm subsystem. There are methods to detect
 * when arm reaches the top and bottom most positions. The intent of these methods is to use them in
 * arm Commands to move the arm up and down but stop when the top and bottom most positions are
 * reached.
 *
 * <p>The encoder is not needed to do any control of the arm moving up or down. It can be used to
 * move the arm to known positions/heights. For example if we plan on picking up pieces from the
 * portal in the substation we should provide a Command that can move the arm to the correct height
 * to pickup cone/cube. We will need to determine what distance/position encoder reads to be at
 * correct height.
 *
 * <p>There is a method to detect a 'middle' position, move a set distance, and to get the current
 * position. The idea here is to create a set of commands that can move the arm to a set position
 * just high enough above a low/mid/high grid scoring node to score a cube/cone. Then provide a
 * second command that can lower the arm and release the gripper to place/score the cube/cone. Not
 * sure this will be useful to the driver. Will need to experiment with this if time allows.
 *
 * <p>There are two limit switches, one at top of range of motion and one at bottom of range of
 * motion. The limit switches are plugged into the roborio digital input/output ports.
 *
 * <p>There is a quadrature encoder on the arm joint. The signal and power lines are plugged into
 * the roborio digital input/output ports.
 */
public class Arm extends ArmBase {

  // Note this value is well above the frame/bumpers, not right above them.  When the arm is moving
  // down there is momentum and time involved.  You need to
  // start closing the gripper and give it time to close before the moving arm is near the
  // frame/bummpers.
  private final double POSITION_GRIPPER_CLOSE = 190;

  /** The constructor for the arm subsystem. */
  public Arm() {
  }

  /**
   * Check if arm is at/nearing position where the gripper needs to be closed. The arm cannot be
   * fully stowed inside robot unless gripper is closed. We do not want to check/return if the
   * position is less than the gripper close position. If we did this, then the gripper would never
   * be able to open while arm is in low positions.
   *
   * @return true if at middle position false if not.
   */
  public boolean atGripperClose() {
    return false;
  }
}
