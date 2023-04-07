// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @since 1/9/2023
 */
public final class Constants {
  /**
   * This is the default distance the robot goes for the autonomous Mobility Routine ({@link
   * bhs.devilbotz.RobotContainer}). The distance is in meters.
   */
  public static final double DEFAULT_DISTANCE_MOBILITY = 5;
  /**
   * This is the default distance the robot goes for the autonomous Dock and Engage Routine ({@link
   * bhs.devilbotz.RobotContainer}). The distance is in meters.
   */
  public static final double DEFAULT_DISTANCE_DOCK_AND_ENGAGE = 2.0;
  /**
   * This the tolerance for the Balance PID Controler. This is used so that the robot does not go on
   * balancing forever but stops when it hits the number listed in this constant. The value is
   * degrees (yaw)
   */
  public static final double BALANCE_PID_TOLERANCE = 3;
  /**
   * Operator constants are for the operator interface (joysticks, buttons, etc.).
   *
   * @since 1/11/2023
   */
  public static class OperatorConstants {
    /**
     * The port for the joystick on the driver station. This is the main joystick and is used for
     * driving
     */
    public static final int DRIVER_LEFT_CONTROLLER_PORT = 0;

    public static final int DRIVER_RIGHT_CONTROLLER_PORT = 1;
  }

  /**
   * Drive constants are for the drive train subsystem characteristics.
   *
   * @since 1/26/2023
   */
  public static class DriveConstants {
    /**
     * The deadband for the joystick. This is the minimum value that the joystick must be at to move
     * the robot.
     */
    public static final double JOYSTICK_DEADBAND = 0.035;

    /** The maximum acceleration of the robot in units per second */
    public static final double SLEW_RATE_LIMITER = 5.5;

    /** The motor configuration for each side of the robot */
    public static final DCMotor MOTOR_CONFIGURATION = DCMotor.getCIM(2);

    /** Drive distance (in meters) from portal to Charge station */
    public static final double POSITION_DRIVE_FROM_PORTAL = 0.75;

    /**
     * Precision rotate speed divider. This is the speed that the robot will rotate at when the
     * driver is toggling the precision rotate button.
     */
    public static final double PRECISION_ROTATE_SPEED_DIVIDER = 2;
  }

  /**
   * Gripper constants are for the gripper subsystem
   *
   * @since 1/30/2023
   */
  public static class GripperConstants {
    /** The gripper double solenoid forward channel */
    public static final int GRIPPER_SOLENOID_FORWARD = 0;
    /** The gripper double solenoid reverse channel */
    public static final int GRIPPER_SOLENOID_REVERSE = 1;
    /** The CAN ID for the gripper's pneumatic compressor */
    public static final int COMPRESSOR_CAN_ID = 6;
  }

  /** SysID dependent constants */
  public static final class SysIdConstants {
    /** Create a linear system from our system identification gains. */
    public static final LinearSystem<N2, N2, N2> PLANT =
        LinearSystemId.identifyDrivetrainSystem(
            Robot.getSysIdConstant("FEED_FORWARD_LINEAR_V").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_LINEAR_A").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_ANGULAR_V").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_ANGULAR_A").asDouble());
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_CAN_ID = 5;
    public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 0;
    public static final int TOP_LIMIT_SWITCH_DIO_PORT = 1;
    public static final int ENCODER_CHANNEL_A_DIO_PORT = 2; // TODO: add what color tape on wire
    public static final int ENCODER_CHANNEL_B_DIO_PORT = 3; // TODO: add what color tape on wire

    public static final double POSITION_TOP = 540; // arm position to score on top goal
    public static final double POSITION_MIDDLE = 450; // arm position to score in mid goal
    public static final double POSITION_BOTTOM = 258; // arm position to score on bottom goal
    public static final double POSITION_TOP_MAX = 600; // arm position at the very top
    public static final double POSITION_CUBE_DELTA =
        -90; // different in score position from cone to cube

    //    public static final double POSITION_PORTAL = 465; // arm position to pickup piece from
    // portal
    public static final double POSITION_DRIVE = 100; // arm position to use when holding a piece
    public static final double POSITION_GRIPPER_CLOSE =
        80; // When the arm is moving down, the positon when the gripper needs to be closed to
    // avoid crashing into the frame
    public static final double POSITION_SCORING_DELTA =
        -20; // The amount to move the gripper before releasing piece
    public static final double POSITION_PICKUP_GROUND_CUBE = 80;
    public static final double POSITION_PICKUP_GROUND_CONE = 0;
    public static final double POSITION_TOLERANCE = 5;
    /**
     * This is the speed to decide if the arm is stuck. If the arm is moving slower that this speed,
     * then it is considered to be stuck
     */
    public static final double SPEED_TO_DECIDE_ARM_STUCK =
        15; /* The arm encoder speed is generally in the +/- 125-175 range */

    public static final double DURATION_TO_DECIDE_ARM_STUCK =
        0.20; /* Wait 40 ms (2 cycle times) before deciding the arm is stuck */
    public static final double ARM_ENCODER_OFFSET =
        -20; /* WORKAROUND: offset to adjust empirically determined arm positions */
    public static final double POSITION_SLOW_ARM = 175;
    public static final double SPEED_DOWN_MAX = 1.0;
    public static final double SPEED_DOWN_SLOW = 0.5;
    public static final double SPEED_UP_MAX = 0.95;
    public static final double SPEED_MANUAL_MAX = 0.5;

    public static final double POSITION_P = 5;
    public static final double POSITION_I = 0;
    public static final double POSITION_D = 0;
  }

  public static final class DebugConstants {
    public static final boolean enableArmMessages = true;
  }
}
