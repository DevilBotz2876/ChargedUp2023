package bhs.devilbotz.subsystems.drive;

import bhs.devilbotz.Robot;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  class DriveIOInputs {
    public double leftDistanceMeters = 0.0;
    public double rightDistanceMeters = 0.0;
    public double leftVelocityMetersPerSec = 0.0;
    public double rightVelocityMetersPerSec = 0.0;
    public double averageDistanceMeters = 0.0;

    public boolean connected = false;
    public double rollPositionDeg = 0.0;
    public double pitchPositionDeg = 0.0;
    public double yawPositionDeg = 0.0;
    public double yawPositionRad = 0.0;
    public double rollVelocityDegPerSec = 0.0;
    public double pitchVelocityDegPerSec = 0.0;
    public double yawVelocityDegPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltages. */
  default void setVoltage(double leftVolts, double rightVolts) {}

  default void resetEncoders() {}

  /**
   * Helper function to convert position (in meters) to Talon SRX encoder native units. Used for
   * Simulation.
   *
   * @param positionMeters The robot's current position (in meters)
   * @return The robot's current position in native units (sensorCount)
   * @see com.ctre.phoenix.motorcontrol.TalonSRXSimCollection#setQuadratureRawPosition(int)
   * @see <a
   *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L208">CTRE
   *     Sample Code</a>
   * @since 1/31/2023
   */
  default int distanceToNativeUnits(double positionMeters) {
    double wheelRotations =
        positionMeters / (2 * Math.PI * Robot.getDriveConstant("WHEEL_RADIUS").asDouble());
    double motorRotations =
        wheelRotations * Robot.getDriveConstant("ENCODER_GEAR_RATIO").asDouble();
    int sensorCounts =
        (int) (motorRotations * Robot.getDriveConstant("ENCODER_RESOLUTION").asInt());
    return sensorCounts;
  }

  /**
   * Helper function to convert velocity to Talon SRX encoder native units. Used for Simulation.
   *
   * @param velocityMetersPerSecond The robot's current velocity (in meter/second)
   * @return The robot's current velocity in native units (encoderCounts per 100ms)
   * @see com.ctre.phoenix.motorcontrol.TalonSRXSimCollection#setQuadratureVelocity(int)
   * @see <a
   *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L215">CTRE
   *     Sample Code</a>
   * @since 1/31/2023
   */
  default int velocityToNativeUnits(double velocityMetersPerSecond) {
    return distanceToNativeUnits(velocityMetersPerSecond) / 10;
  }

  /**
   * Helper function to convert Talon SRX sensor counts to meters. Used for Simulation.
   *
   * @param sensorCounts The robot's encoder count
   * @return The robot's current position in meters
   * @see com.ctre.phoenix.motorcontrol.TalonSRXSimCollection#setQuadratureVelocity(int)
   * @see <a
   *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L223">CTRE
   *     Sample Code</a>
   * @since 1/31/2023
   */
  default double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations =
        (double) sensorCounts / Robot.getDriveConstant("ENCODER_RESOLUTION").asInt();
    double wheelRotations =
        motorRotations / Robot.getDriveConstant("ENCODER_GEAR_RATIO").asDouble();
    double positionMeters =
        wheelRotations * (2 * Math.PI * Robot.getDriveConstant("WHEEL_RADIUS").asDouble());
    return positionMeters;
  }

  /**
   * Helper function to convert Talon SRX sensor counts per 100ms to meters/second. Used for
   * Simulation.
   *
   * @param sensorCountsPer100ms The robot's encoder count per 100ms
   * @return The robot's current velocity in meters per second
   * @see com.ctre.phoenix.motorcontrol.TalonSRXSimCollection#setQuadratureVelocity(int)
   * @see <a
   *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L223">CTRE
   *     Sample Code</a>
   * @since 1/31/2023
   */
  default double nativeUnitsToVelocityMetersPerSecond(double sensorCountsPer100ms) {
    return nativeUnitsToDistanceMeters(10 * sensorCountsPer100ms);
  }
}
