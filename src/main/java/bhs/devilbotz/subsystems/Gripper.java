// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  /** The constructor for the gripper subsystem. */
  public Gripper() {
    pneumaticHub.disableCompressor();
  }

  /** This method opens the gripper. */
  public void open() {
    doubleSolenoid.set(Value.kForward);
    System.out.println("Opening");
  }

  /** This method closes the gripper. */
  public void close() {
    doubleSolenoid.set(Value.kReverse);
    System.out.println("Closing");
  }

  /** This method sets the grippers speed to 0. */
  public void stop() {
    doubleSolenoid.set(Value.kOff);
    System.out.println("Stopping");
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
