package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  // Define talons
  private static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(4);
  private static final WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
  private static final WPI_TalonSRX rightFollower = new WPI_TalonSRX(2);

  // Define NAVX
  private static final AHRS navx = new AHRS(SPI.Port.kMXP);

  // Define differential drive
  private final DifferentialDrive differentialDrive =
      new DifferentialDrive(leftMaster, rightMaster);

  private final PIDController leftPIDController = new PIDController(Constants.DriveConstants.driveP,
          Constants.DriveConstants.driveI, Constants.DriveConstants.driveD);
  private final PIDController rightPIDController = new PIDController(Constants.DriveConstants.driveP,
          Constants.DriveConstants.driveI, Constants.DriveConstants.driveD);

  private final DifferentialDriveKinematics kinematics =
          new DifferentialDriveKinematics(Constants.DriveConstants.trackWidth);

  private final DifferentialDriveOdometry odometry;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.driveFFS,
          Constants.DriveConstants.driveFFV, Constants.DriveConstants.driveFFA);

  private final Field2d field = new Field2d();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    setupTalons();
    resetNavx();
    resetEncoders();

    SmartDashboard.putData("Field", field);

    odometry =
            new DifferentialDriveOdometry(
                    navx.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());
  }

  /**
   * Get the left encoder distance.
   * @return The left encoder distance in meters.
   * @see #getRightDistance()
   */
  private double getLeftDistance() {
    SmartDashboard.putNumber("LeftDistance", leftMaster.getSelectedSensorPosition() * (2 * Math.PI * Constants.DriveConstants.wheelRadius / Constants.DriveConstants.encoderResolution));
    return leftMaster.getSelectedSensorPosition() * (2 * Math.PI * Constants.DriveConstants.wheelRadius / Constants.DriveConstants.encoderResolution);
  }

  /**
   * Get the right encoder distance.
   * @return The right encoder distance in meters.
   * @see #getLeftDistance()
   */
  private double getRightDistance() {
    return rightMaster.getSelectedSensorPosition() * (2 * Math.PI * Constants.DriveConstants.wheelRadius / Constants.DriveConstants.encoderResolution);
  }

  /**
   * Get the left encoder velocity.
   * @return The left encoder velocity in meters per second.
   * @see #getRightVelocity()
   */
    private double getLeftVelocity() {
      return leftMaster.getSelectedSensorVelocity() * (2 * Math.PI * Constants.DriveConstants.wheelRadius / Constants.DriveConstants.encoderResolution);
    }

    /**
     * Get the right encoder velocity.
     * @return The right encoder velocity in meters per second.
     * @see #getLeftVelocity()
     */
    private double getRightVelocity() {
      return rightMaster.getSelectedSensorVelocity() * (2 * Math.PI * Constants.DriveConstants.wheelRadius / Constants.DriveConstants.encoderResolution);
    }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds in meters per second.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    // get rate is meters per second
    final double leftOutput =
            leftPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
            rightPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);
    leftMaster.setVoltage(leftOutput + leftFeedforward);
    SmartDashboard.putNumber("leftOutput", leftOutput + leftFeedforward);
    System.out.println(leftFeedforward);
    System.out.println("output: " + leftOutput);
    rightMaster.setVoltage(rightOutput + rightFeedforward);
    SmartDashboard.putNumber("rightOutput", rightOutput + rightFeedforward);

  }

  private void setupTalons() {
    rightMaster.setInverted(true);
    leftMaster.setInverted(false);
    // Set the talons to follow each other
    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    // Set the follower talons to invert to match the master talons
    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);

    this.setTalonMode(NeutralMode.Brake);

    // Set the sensor phase of the master talons
    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);
  }

  /**
   * Reset the NAVX
   *
   * @since 1.0.0
   */
  public void resetNavx() {
    navx.reset();
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param speed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void arcadeDrive(double speed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void setTalonMode(NeutralMode mode) {
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    odometry.update(
            navx.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public WPI_TalonSRX getLeftMaster() {
    return leftMaster;
  }

  public WPI_TalonSRX getRightMaster() {
    return rightMaster;
  }

  public WPI_TalonSRX getLeftFollower() {
    return leftFollower;
  }

  public WPI_TalonSRX getRightFollower() {
    return rightFollower;
  }

  public AHRS getNavx() {
    return navx;
  }

  public double getRoll() {
    return navx.getRoll();
  }
}
