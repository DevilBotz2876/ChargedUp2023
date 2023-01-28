// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

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
    public static final double maxSpeed = 2.447; // meters per second TODO: Measure/SysID
    public static final double maxAngularSpeed = 2 * Math.PI; // one rotation per second TODO: Measure

    public static final double trackWidth = 0.555; // meters, robot width TODO: Measure
    public static final double wheelRadius = 0.0762; // meters (Andymark am-0940b "High Grip Wheels, 6")
    public static final int encoderResolution = 4096; // CTRE Magnetic Encoder

    // Drive PID values TODO: Tune
    public static final double driveP = 0.39941;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    // Feedforward constants TODO: Tune
    public static final double driveFFS = 1.032;
    public static final double driveFFV = 2.4523;
    public static final double driveFFA = 0.28226;
  }
}
