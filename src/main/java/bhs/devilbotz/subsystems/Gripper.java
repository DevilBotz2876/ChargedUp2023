// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.utils.RobotConfig;
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
  private static final PneumaticHub pneumaticHub = null;
  private final DoubleSolenoid doubleSolenoid = null;

  /** The constructor for the gripper subsystem. */
  public Gripper() {
    if (RobotConfig.isCompBot()) {
      pneumaticHub.disableCompressor();
      pneumaticHub = new PneumaticHub(Constants.GripperConstants.COMPRESSOR_CAN_ID);
      doubleSolenoid =
        pneumaticHub.makeDoubleSolenoid(
          Constants.GripperConstants.GRIPPER_SOLENOID_FORWARD,
          Constants.GripperConstants.GRIPPER_SOLENOID_REVERSE);
    }
  }

  /** This method opens the gripper. */
  public void open() {
    if (RobotConfig.isCompBot()) {
      doubleSolenoid.set(Value.kForward);
    }
  }

  /** This method closes the gripper. */
  public void close() {
    if (RobotConfig.isCompBot()) {
      doubleSolenoid.set(Value.kReverse);
    }
  }

  /** This method sets the grippers speed to 0. */
  public void stop() {
    if (RobotConfig.isCompBot()) {
      doubleSolenoid.set(Value.kOff);
    }
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
  public void enableCompressor() {
    if (RobotConfig.isCompBot()) {
      if (!pneumaticHub.getCompressor()) {
        pneumaticHub.enableCompressorDigital();
      }
    }
  }

  /**
   * Returns true if the compressor pressure has reached the set value. The set value is controlled
   * physically on the robot
   *
   * @return true if pressue is at pre-configured set point
   */
  public boolean getAtSetpoint() {
    if (RobotConfig.isCompBot()) {
      return pneumaticHub.getPressureSwitch();
    }
    return false;
  }
}
