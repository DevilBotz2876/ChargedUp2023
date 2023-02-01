// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class Gripper extends SubsystemBase {
  DoubleSolenoid gripperSolenoid;

  /**
   * The constructor for the gripper subsystem.
   */
  public Gripper() {
    gripperSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.GripperConstants.GRIPPER_SOLENOID_FORWARD,
            Constants.GripperConstants.GRIPPER_SOLENOID_REVERSE);
  }

  /**
   * This method opens the gripper.
   */
  public void open() {
    gripperSolenoid.set(Value.kForward);
  }

  /**
   * This method closes the gripper.
   */
  public void close() {
    gripperSolenoid.set(Value.kReverse);
  }

  /**
   * This method sets the grippers speed to 0.
   */
  public void stop() {
    gripperSolenoid.set(Value.kOff);
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
