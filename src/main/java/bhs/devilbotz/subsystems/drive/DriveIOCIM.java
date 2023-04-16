package bhs.devilbotz.subsystems.drive;

import bhs.devilbotz.Robot;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveIOCIM implements DriveIO {
  private static final WPI_TalonSRX leftMaster =
      new WPI_TalonSRX(Robot.getDriveConstant("MOTOR_LEFT_MASTER_CAN_ID").asInt());
  private static final WPI_TalonSRX rightMaster =
      new WPI_TalonSRX(Robot.getDriveConstant("MOTOR_RIGHT_MASTER_CAN_ID").asInt());
  private static final WPI_TalonSRX leftFollower =
      new WPI_TalonSRX(Robot.getDriveConstant("MOTOR_LEFT_FOLLOWER_CAN_ID").asInt());
  private static final WPI_TalonSRX rightFollower =
      new WPI_TalonSRX(Robot.getDriveConstant("MOTOR_RIGHT_FOLLOWER_CAN_ID").asInt());

  private final AHRS navx;

  public DriveIOCIM() {
    navx = new AHRS(SPI.Port.kMXP);

    navx.reset();

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Inverts the right side of the drive train
    rightMaster.setInverted(true);
    leftMaster.setInverted(false);

    // Set the talons to follow each other
    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    // Set the follower talons to invert to match the master talons
    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    if (Robot.checkCapability("hasInvertedDriveEncoderSensorPhase")) {
      rightMaster.setSensorPhase(true);
      leftMaster.setSensorPhase(true);
    } else {
      rightMaster.setSensorPhase(false);
      leftMaster.setSensorPhase(false);
    }

    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Updates the set of loggable inputs.
    inputs.leftDistanceMeters = getLeftDistance();
    inputs.rightDistanceMeters = getRightDistance();
    inputs.leftVelocityMetersPerSec = getLeftVelocity();
    inputs.rightVelocityMetersPerSec = getRightVelocity();
    inputs.averageDistanceMeters = getAverageDistance();

    inputs.connected = navx.isConnected();
    inputs.rollPositionDeg = navx.getRoll();
    inputs.pitchPositionDeg = navx.getPitch();
    inputs.yawPositionDeg = navx.getYaw();
    inputs.yawPositionRad = Math.toRadians(navx.getYaw());
    inputs.rollVelocityDegPerSec = navx.getRawGyroX();
    inputs.pitchVelocityDegPerSec = navx.getRawGyroY();
    inputs.yawVelocityDegPerSec = navx.getRawGyroZ();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    // Sets the motor controller speeds.
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  /**
   * Get the left encoder distance.
   *
   * @return The left encoder distance in meters.
   * @see #getRightDistance()
   * @since 1/30/2023
   */
  private double getLeftDistance() {
    return nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition());
  }

  /**
   * Get the right encoder distance.
   *
   * @return The right encoder distance in meters.
   * @see #getLeftDistance()
   * @since 1/30/2023
   */
  private double getRightDistance() {
    return nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition());
  }

  /**
   * Get the average distance of both encoders
   *
   * @return the average distance of both encoders in meters
   */
  private double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }
  /**
   * Get the left encoder velocity.
   *
   * @return The left encoder velocity in meters per second.
   * @see #getRightVelocity()
   */
  private double getLeftVelocity() {
    return nativeUnitsToVelocityMetersPerSecond(leftMaster.getSelectedSensorVelocity());
  }

  /**
   * Get the right encoder velocity.
   *
   * @return The right encoder velocity in meters per second.
   * @see #getLeftVelocity()
   */
  private double getRightVelocity() {
    return nativeUnitsToVelocityMetersPerSecond(rightMaster.getSelectedSensorVelocity());
  }

  @Override
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }
}
