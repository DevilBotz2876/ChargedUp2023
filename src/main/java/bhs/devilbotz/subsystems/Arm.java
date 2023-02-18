// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj &amp; ParkerMeyers
 */
public class Arm extends SubsystemBase {
  private final CANSparkMax armMotor;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  private final Encoder encoder;

  /** The constructor for the arm subsystem. */
  public Arm() {
    armMotor =
        new CANSparkMax(
            Constants.ArmConstants.ARM_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    topLimitSwitch = new DigitalInput(1);
    bottomLimitSwitch = new DigitalInput(0);
    encoder = new Encoder(2, 3);

    // TODO: when arm is fixed try using this to change numbers output by encoder.  With this set to
    // true moving up will increase encoder value.  Moving down will decrease values.

    // Nicer on the eyes to see positive numbers when moving up/down.
    // encoder.setReverseDirection(true);
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("armTopLimit", isTopLimit());
    SmartDashboard.putBoolean("armBotLimit", isBottomLimit());
    SmartDashboard.putNumber("armPosition", encoder.getDistance());

    // TODO: when arm is fixed change this to reset encoder position when at bottom
    if (isTopLimit()) {
      encoder.reset();
    }
  }

  /**
   * This method sets the speed of the arm.
   *
   * @param speed The speed of the arm.
   */
  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  public void up() {
    armMotor.set(1);
  }

  public void down() {
    armMotor.set(-1);
  }

  public boolean atBottom() {
    // TODO: when arm is fixed change this to check bottom limit switch
    //
    // return isBottomLimit();
    return encoder.getDistance() >= 400;
  }

  public boolean atTop() {
    return isTopLimit();
  }

  public boolean atMiddle() {
    // Just a random number for now.  Need to measure what real/good encoder value is to position
    // arm right above middle scoring spot.
    return encoder.getDistance() >= 200 && encoder.getDistance() <= 205;
  }

  /** This method stops the arm. */
  public void stop() {
    armMotor.set(0);
    armMotor.stopMotor();
  }

  public boolean isTopLimit() {
    return !topLimitSwitch.get();
  }

  public boolean isBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public double getPosition() {
    return encoder.getDistance();
  }
}
