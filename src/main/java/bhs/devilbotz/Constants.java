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

  public static final double BALANCE_P = 0.07; // TODO: Get correct values
  public static final double BALANCE_I = 0.0; // TODO: Get correct values
  public static final double BALANCE_D = 0.01; // TODO: Get correct values

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class DriveConstants {
    public static final int kMotorPortLeftMaster = 3;
    public static final int kMotorPortRightMaster = 4;
    public static final int kMotorPortLeftFollower = 1;
    public static final int kMotorPortRightFollower = 2;
  }
}
