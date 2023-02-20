// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Constants.ArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    topLimitSwitch = new DigitalInput(ArmConstants.TOP_LIMIT_SWITCH_DIO_PORT);
    bottomLimitSwitch = new DigitalInput(ArmConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT);
    encoder =
        new Encoder(
            ArmConstants.ENCODER_CHANNEL_A_DIO_PORT, ArmConstants.ENCODER_CHANNEL_B_DIO_PORT);

    // TODO: when arm is fixed try using this to change numbers output by encoder.  ill increase
    // encoder value.  Moving down will decrease values.
    //
    // Arm starts down inside chassis. This means encoder starts at zero here.  When moving the arm
    // up it is
    // nice to see positive numbers for position.  With this set to
    // true moving arm up from initial position should return positive value.  TODO: check if
    // switching DIO port wires will change sign of value.
    //
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

  /** Move the arm up at set speed. There is no check/protection against moving arm too far up. */
  public void up() {
    armMotor.set(1);
  }

  /**
   * Move the arm down at set speed. There is no check/protection against moving arm too far down.
   */
  public void down() {
    armMotor.set(-1);
  }

  /**
   * Check if arm is at bottom most position.
   *
   * @return true if at bottom, false if not.
   */
  public boolean atBottom() {
    // TODO: when arm is fixed change this to check bottom limit switch
    //
    // return isBottomLimit();
    return encoder.getDistance() >= 400;
  }

  /**
   * Check if arm is at top most position.
   *
   * @return true if at top, false if not.
   */
  public boolean atTop() {
    return isTopLimit();
  }

  /** This method stops the arm. */
  public void stop() {
    armMotor.set(0);
    armMotor.stopMotor();
  }

  /**
   * Return state of top limit switch
   *
   * @return true limit switch is pressed, false if not.
   */
  public boolean isTopLimit() {
    return !topLimitSwitch.get();
  }

  /**
   * Return state of bottom limit switch
   *
   * @return true limit switch is pressed, false if not.
   */
  public boolean isBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  /**
   * Check if arm is at middle position based on encoder position. Middle is just high enough above
   * middle grid scoring node to allow cone or cube to be lined up.
   *
   * @return true if at middle position false if not.
   */
  public boolean atMiddle() {
    // TODO: Just a random number for now.  Need to measure what real/good encoder value is to
    // position arm right above middle scoring grid node.
    return encoder.getDistance() >= 200 && encoder.getDistance() <= 205;
  }

  /**
   * Check if arm is at correct height to grab cone/cube from substation portal.
   *
   * @return true if at substation portal height, false if not.
   */
  public boolean atSubstationPortal() {
    // TODO: Just a random number for now.  Need to measure what real/good encoder value is to
    // position
    // arm at height to pickup from portal
    return encoder.getDistance() >= 200 && encoder.getDistance() <= 205;
  }

  /**
   * Return current position of arm.
   *
   * @return position of arm.
   */
  public double getPosition() {
    return encoder.getDistance();
  }
}
