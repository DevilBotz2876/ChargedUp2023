package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants.ArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmBase extends SubsystemBase {
  private final CANSparkMax armMotor;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  private final Encoder encoder;

  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("Arm");

  private BooleanEntry ntTopLimitSwitch = table.getBooleanTopic("limit/top").getEntry(false);
  private BooleanEntry ntBottomLimitSwitch = table.getBooleanTopic("limit/bottom").getEntry(false);

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

    // Only reset encoder position if arm hit bottom limit switch and is not moving.  The arm will
    // be engaged with limit switch while moving.  We don't want to reset encoder multiple times.
    if (isBottomLimit() && isMoving() == false) {
      resetPosition();
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

  /** This method stops the arm. */
  public void stop() {
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

  public DigitalInput getBottomLimitSwitch() {
    return bottomLimitSwitch;
  }

  public DigitalInput getTopLimitSwitch() {
    return topLimitSwitch;
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

  public Encoder getEncoder() {
    return encoder;
  }
}
