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
 */
public final class Constants {

  public static final int JOYSTICK_PORT = 0;

  public static final double BALANCE_P = 0.39941; // TODO: Get correct values
  public static final double BALANCE_I = 0.0; // TODO: Get correct values
  public static final double BALANCE_D = 0.00; // TODO: Get correct values

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class DriveConstants {
    public static final double SLEW_RATE_LIMITER = 3;
    public static final double MAX_SPEED = 2.447; // meters per second TODO: Measure/SysID
    public static final double MAX_ANGULAR_SPEED =
        2 * Math.PI; // one rotation per second TODO: Measure

    public static final double TRACK_WIDTH = 0.555; // meters, robot width TODO: Measure
    public static final double WHEEL_RADIUS =
        0.0762; // meters (Andymark am-0940b "High Grip Wheels, 6")
    public static final int ENCODER_RESOLUTION = 4096; // CTRE Magnetic Encoder
    public static final int ENCODER_GEAR_RATIO = 1; // Encoder is connected directly to drive shaft

    // Drive PID values TODO: Tune
    public static final double DRIVE_P = 3.5725;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    // Feedforward constants (straight-line) TODO: Tune
    public static final double DRIVE_FFS = 0.94143;
    public static final double DRIVE_FFV = 2.3803;
    public static final double DRIVE_FFA = 0.48128;

    // Feedforward constants (rotating) TODO: Tune
    public static final double DRIVE_ANGULAR_FFS = 1.5;
    public static final double DRIVE_ANGULAR_FFV = 3;
    public static final double DRIVE_ANGULAR_FFA = 0.3;

    // Create a linear system from our system identification gains.
    public static final LinearSystem<N2, N2, N2> DRIVE_PLANT =
        LinearSystemId.identifyDrivetrainSystem(
            DRIVE_FFV, DRIVE_FFA, DRIVE_ANGULAR_FFV, DRIVE_ANGULAR_FFA);

    // Dual Talon SRX CIM motors on each side of drive train
    public static final DCMotor MOTOR_CONFIGURATION = DCMotor.getCIM(2);

    public static final int MOTOR_LEFT_MASTER_CAN_ID = 3;
    public static final int MOTOR_RIGHT_MASTER_CAN_ID = 4;
    public static final int MOTOR_LEFT_FOLLOWER_CAN_ID = 1;
    public static final int MOTOR_RIGHT_FOLLOWER_CAN_ID = 2;

    public static final double MOTOR_GEAR_RATIO = 8.45;
  }

  public static class GripperConstants {
    public static final int GRIPPER_SOLENOID_FORWARD = 0;
    public static final int GRIPPER_SOLENOID_REVERSE = 1;
  }
}
