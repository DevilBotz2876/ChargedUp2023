// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  DoubleSolenoid gripperSolenoid;
  /** Creates a new Gripper. */
  public Gripper() {
    gripperSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.GripperConstants.GRIPPER_SOLENOID_FORWARD,
            Constants.GripperConstants.GRIPPER_SOLENOID_REVERSE);
  }

  public void open() {
    gripperSolenoid.set(Value.kForward);
  }

  public void close() {
    gripperSolenoid.set(Value.kReverse);
  }

  public void stop() {
    gripperSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
