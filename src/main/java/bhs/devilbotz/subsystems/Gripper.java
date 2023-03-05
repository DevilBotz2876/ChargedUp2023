// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj &amp; ParkerMeyers
 */
public class Gripper extends SubsystemBase {
  private static final PneumaticHub pneumaticHub =
      new PneumaticHub(Constants.GripperConstants.COMPRESSOR_CAN_ID);

  private final DoubleSolenoid doubleSolenoid =
      pneumaticHub.makeDoubleSolenoid(
          Constants.GripperConstants.GRIPPER_SOLENOID_FORWARD,
          Constants.GripperConstants.GRIPPER_SOLENOID_REVERSE);

  // Network Table Based Debug Status
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("Gripper");
  private StringEntry ntState = table.getStringTopic("state").getEntry("Unknown");

  /** The constructor for the gripper subsystem. */
  public Gripper() {
    pneumaticHub.disableCompressor();
    ntState.set("Unknown");
    SmartDashboard.putData("HW/Gripper/Solenoid", doubleSolenoid);
  }

  /** This method opens the gripper. */
  public void open() {
    doubleSolenoid.set(Value.kForward);
    ntState.set("Open");
  }

  /** This method closes the gripper. */
  public void close() {
    doubleSolenoid.set(Value.kReverse);
    ntState.set("Closed");
  }

  /**
   * This method deactivates the gripper solenoid. It should be ok to leave the solenoid in
   * fwd/reverse. Reading thru this thread on CD explains why:
   * https://www.chiefdelphi.com/t/do-you-need-to-set-a-double-solenoid-to-off-after-bringing-it-in/368798
   * Based on comments in that thread we should be fine leaving the solenoid actuated/firing all the
   * time. We want do do this to prevent the gripper from flapping if the gate in the solenoid
   * moves/slips while the robot is moving around. There is also a delay involved to make sure the
   * solenoid fired before disabling it. This makes the code more complicated. Ff we can get by with
   * just leaving the solenoid firing all the time we should do so.
   */
  public void stop() {
    doubleSolenoid.set(Value.kOff);
    ntState.set("Stopped");
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {}

  /** Enables the compressor for the pnuematic gripper. Remains on until the robot is disabled. */
  public static void enableCompressor() {
    if (!pneumaticHub.getCompressor()) {
      pneumaticHub.enableCompressorDigital();
    }
  }

  /**
   * Returns true if the compressor pressure has reached the set value. The set value is controlled
   * physically on the robot
   *
   * @return true if pressue is at pre-configured set point
   */
  public static boolean getAtSetpoint() {
    return pneumaticHub.getPressureSwitch();
  }
}
