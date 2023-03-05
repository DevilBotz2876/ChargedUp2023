package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.Robot;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmBase extends SubsystemBase {
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

  public ArmBase() {
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
          new SingleJointedArmSim(armGearbox, 400, .1, armLength, minAngle, maxAngle, false, null);
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
  public void up() {
    ntState.set("Moving: Up");
    armMotor.set(1);
  }

  /**
   * Move the arm down at set speed. There is no check/protection against moving arm too far down.
   */
  public void down() {
    ntState.set("Moving: Down");
    armMotor.set(-1);
  }

  /** This method stops the arm. */
  public void stop() {
    ntState.set("Stopped");
    armMotor.set(0.0);
    armMotor.stopMotor();
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

  /**
   * Reset encoder to zero. We consider arm down inside chassis as zero position. Use lower limit
   * switch to determine when we reach that position. This should be called when arm is not moving.
   */
  protected void resetPosition() {
    encoder.reset();
  }
}
