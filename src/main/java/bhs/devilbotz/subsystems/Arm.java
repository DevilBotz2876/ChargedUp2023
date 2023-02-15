// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import bhs.devilbotz.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj &amp; ParkerMeyers
 */
public class Arm extends SubsystemBase {

  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  /** The constructor for the arm subsystem. */
  public Arm() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {}

  /**
   * This method sets the speed of the arm.
   *
   * @param speed The speed of the arm.
   */
  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  /** This method stops the arm. */
  public void stop() {
    armMotor.set(0);
    armMotor.stopMotor();
  }

  // TODO: ADD LIMIT SWITCHES
}
