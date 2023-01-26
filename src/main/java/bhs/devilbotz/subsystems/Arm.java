// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX armMotor;

  private boolean armUp;

  public Arm() {
    armMotor = new WPI_TalonSRX(Constants.ARM_CAN_ID);
    armUp = false;
  }

  public boolean isArmUp() {
    return armUp;
  }

  public void Up(double speed) {
    armMotor.set(speed);
    armUp = true;
  }

  public void Down(double speed) {
    armMotor.set(-speed);
    armUp = false;
  }

  public void stop() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
