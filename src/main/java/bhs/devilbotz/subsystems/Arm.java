// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.RobotContainer;
import bhs.devilbotz.lib.LEDModes;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
 * <p>There is a method to get the current position. The idea here is to create a set of commands
 * that can move the arm to a set position just high enough above a low/mid/high grid scoring node
 * to score a cube/cone. Then provide a second command that can lower the arm and release the
 * gripper to place/score the cube/cone. Not sure this will be useful to the driver. Will need to
 * experiment with this if time allows.
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

  // Simulation Variables
  // create a sim controller for the encoder
  EncoderSim encoderSim;
  DIOSim topLimitSwitchSim;
  DIOSim bottomLimitSwitchSim;
  SingleJointedArmSim armSim;

  // Network Table Based Debug Status
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("Arm");

  private BooleanEntry ntTopLimitSwitch = table.getBooleanTopic("limit/top").getEntry(false);
  private BooleanEntry ntBottomLimitSwitch = table.getBooleanTopic("limit/bottom").getEntry(false);
  private DoubleEntry ntPosition = table.getDoubleTopic("position").getEntry(0);
  private StringEntry ntState = table.getStringTopic("state").getEntry("Unknown");

  private RobotContainer robotContainer;

  public Arm(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    armMotor =
        new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    topLimitSwitch = new DigitalInput(ArmConstants.TOP_LIMIT_SWITCH_DIO_PORT);
    bottomLimitSwitch = new DigitalInput(ArmConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT);

    encoder =
        new Encoder(
            ArmConstants.ENCODER_CHANNEL_A_DIO_PORT, ArmConstants.ENCODER_CHANNEL_B_DIO_PORT);

    // Arm starts down inside chassis. This means encoder starts at zero here.  When moving the arm
    // up it is nice to see positive numbers for position.  With this set to
    // true moving arm up from initial position should return positive value.
    //
    // TODO: check if switching DIO port wires will change sign of value.
    //
    encoder.setReverseDirection(true);

    SmartDashboard.putData("HW/Arm/limit/top", topLimitSwitch);
    SmartDashboard.putData("HW/Arm/limit/bottom", bottomLimitSwitch);
    SmartDashboard.putData("HW/Arm/encoder", encoder);
    setupSimulation();
  }

  void setupSimulation() {
    if (Robot.isSimulation()) {
      encoderSim = new EncoderSim(encoder);
      encoderSim.setCount(0);

      topLimitSwitchSim = new DIOSim(topLimitSwitch);
      bottomLimitSwitchSim = new DIOSim(bottomLimitSwitch);
      topLimitSwitchSim.setValue(false);
      bottomLimitSwitchSim.setValue(false);

      DCMotor armGearbox = DCMotor.getNEO(1);
      double armLength = Units.inchesToMeters(48);
      double minAngle = Units.degreesToRadians(0);
      double maxAngle = Units.degreesToRadians(100);
      armSim =
          new SingleJointedArmSim(armGearbox, 2000, 1, armLength, minAngle, maxAngle, false, null);
    }
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {
    ntTopLimitSwitch.set(isTopLimit());
    ntBottomLimitSwitch.set(isBottomLimit());
    ntPosition.set(getPosition());

    // Only reset encoder position if arm hit bottom limit switch and is not moving.  The arm will
    // be engaged with limit switch while moving.  We don't want to reset encoder multiple times.
    if (isBottomLimit() && isMoving() == false) {
      resetPosition();
    }
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Set our simulated encoder's readings.  The range of movement setup in sim results in a range
    // of 0-200 degrees.
    //
    double distance = 8 * (Units.radiansToDegrees(armSim.getAngleRads()));
    encoderSim.setDistance(distance);

    topLimitSwitchSim.setValue(!armSim.hasHitUpperLimit());
    bottomLimitSwitchSim.setValue(!armSim.hasHitLowerLimit());

    // Update the Mechanism Arm angle based on the simulated arm angle
    //    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
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
  public void up(boolean slow) {
    ntState.set("Moving: Up");
    if (slow) armMotor.set(0.5);
    else armMotor.set(0.9);
    robotContainer.setLEDMode(LEDModes.SET_ARM_UP);
  }

  /**
   * Move the arm down at set speed. There is no check/protection against moving arm too far down.
   */
  public void down(boolean slow) {
    ntState.set("Moving: Down");
    if (slow) armMotor.set(-0.5);
    else armMotor.set(-0.9);
    robotContainer.setLEDMode(LEDModes.SET_ARM_DOWN);
  }

  /** This method stops the arm. */
  public void stop() {
    ntState.set("Stopped");
    armMotor.set(0.0);
    armMotor.stopMotor();
    robotContainer.setLEDMode(LEDModes.SET_ARM_IDLE);
  }

  /**
   * Check if arm is moving or not based on motor state.
   *
   * @return true if arm is moving, false if not.
   */
  public boolean isMoving() {
    return (armMotor.get() != 0.0);
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
   * Return current position of arm.
   *
   * @return position of arm.
   */
  public double getPosition() {
    return encoder.getDistance();
  }

  public double getVelocity() {
    return encoder.getRate();
  }

  /**
   * Reset encoder to zero. We consider arm down inside chassis as zero position. Use lower limit
   * switch to determine when we reach that position. This should be called when arm is not moving.
   */
  protected void resetPosition() {
    encoder.reset();
  }
}
