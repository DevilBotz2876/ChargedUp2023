package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.RobotContainer;
import bhs.devilbotz.utils.ShuffleboardManager;
import bhs.devilbotz.Constants.DriveConstants;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The DriveTrain subsystem controls the robot's drive train. It also handles: - The NAVX
 * (gyroscope) - The odometry (position tracking) - The kinematics (wheel speeds)
 *
 * @author ParkerMeyers
 * @see <a
 *     href="https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/index.html">Kinematics
 *     and Odometry</a>
 * @since 1/30/2023
 */
public class DriveTrain extends SubsystemBase {
  // Defines the motor controllers for both sides of the drive train (left and right).

  private static final WPI_TalonSRX leftMaster =
      new WPI_TalonSRX(DriveConstants.MOTOR_LEFT_MASTER_CAN_ID);
  private static final WPI_TalonSRX rightMaster =
      new WPI_TalonSRX(DriveConstants.MOTOR_RIGHT_MASTER_CAN_ID);
  private static final WPI_TalonSRX leftFollower =
      new WPI_TalonSRX(DriveConstants.MOTOR_LEFT_FOLLOWER_CAN_ID);
  private static final WPI_TalonSRX rightFollower =
      new WPI_TalonSRX(DriveConstants.MOTOR_RIGHT_FOLLOWER_CAN_ID);

  // Defines the NAVX, which is a gyroscope that we use to track the robot's position.
  // It is attached to the robot via SPI (Serial Peripheral Interface).
  private static final AHRS navx = new AHRS(SPI.Port.kMXP);

  // Defines the PID controllers for the left and right sides of the drive train.
  // These are used to control the speed of the motors proportionally to the speed of the wheels.
  private final PIDController leftPIDController =
      new PIDController(
          Constants.DriveConstants.DRIVE_P,
          Constants.DriveConstants.DRIVE_I,
          Constants.DriveConstants.DRIVE_D);
  private final PIDController rightPIDController =
      new PIDController(
          Constants.DriveConstants.DRIVE_P,
          Constants.DriveConstants.DRIVE_I,
          Constants.DriveConstants.DRIVE_D);

  // Defines the kinematics of the drive train, which is used to calculate the speed of the wheels.
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);

  // Defines the odometry of the drive train, which is used to calculate the position of the robot.
  private final DifferentialDriveOdometry odometry;

  // Defines the feedforward of the drive train, which is used to calculate the voltage needed to
  // move the robot.
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.DriveConstants.DRIVE_FFS,
          Constants.DriveConstants.DRIVE_FFV,
          Constants.DriveConstants.DRIVE_FFA);

  // Defines the field, which is used to display the robot's position on the field in Shuffleboard.
  private final Field2d field = new Field2d();

  private final ShuffleboardManager shuffleboardManager;

  /**
   * DriveTrain constructor This constructor sets up the drive train.
   *
   * @since 1/30/2023
   */
  public DriveTrain(RobotContainer robotContainer) {
    this.shuffleboardManager = robotContainer.getShuffleboardManager();

    // Sets the motor controllers to the correct mode & inverts the right side
    setupTalons();
    // Sets the initial position of the robot to (0, 0) and the initial angle to 0 degrees.
    resetNavx();
    resetEncoders();

    // Adds the field to Shuffleboard.
    shuffleboardManager.putField(field);

    // Defines the odometry of the drive train, which is used to calculate the position of the
    // robot.
    odometry =
        new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   * @since 1/30/2023
   */
  @Override
  public void periodic() {
    // Updates the odometry of the drive train.
    updateOdometry();
  }

  /**
   * Get the left encoder distance.
   *
   * @return The left encoder distance in meters.
   * @see #getRightDistance()
   * @since 1/30/2023
   */
  private double getLeftDistance() {
    return leftMaster.getSelectedSensorPosition()
        * (2
            * Math.PI
            * Constants.DriveConstants.WHEEL_RADIUS
            / Constants.DriveConstants.ENCODER_RESOLUTION);
  }

  /**
   * Get the right encoder distance.
   *
   * @return The right encoder distance in meters.
   * @see #getLeftDistance()
   * @since 1/30/2023
   */
  private double getRightDistance() {
    return rightMaster.getSelectedSensorPosition()
        * (2
            * Math.PI
            * Constants.DriveConstants.WHEEL_RADIUS
            / Constants.DriveConstants.ENCODER_RESOLUTION);
  }

  /**
   * Get the left encoder velocity.
   *
   * @return The left encoder velocity in meters per second.
   * @see #getRightVelocity()
   */
  private double getLeftVelocity() {
    return leftMaster.getSelectedSensorVelocity()
        * (2
            * Math.PI
            * Constants.DriveConstants.WHEEL_RADIUS
            / Constants.DriveConstants.ENCODER_RESOLUTION);
  }

  /**
   * Get the right encoder velocity.
   *
   * @return The right encoder velocity in meters per second.
   * @see #getLeftVelocity()
   */
  private double getRightVelocity() {
    return rightMaster.getSelectedSensorVelocity()
        * (2
            * Math.PI
            * Constants.DriveConstants.WHEEL_RADIUS
            / Constants.DriveConstants.ENCODER_RESOLUTION);
  }

  /**
   * Sets the desired wheel speeds using the PID controllers.
   *
   * @param speeds The desired wheel speeds in meters per second.
   * @since 1/30/2023
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculates the desired voltages for the left and right sides of the drive train.
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    // Calculates the PID output for the left and right sides of the drive train.
    final double leftOutput =
        leftPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    // Sets the motor controller speeds.
    leftMaster.setVoltage(leftOutput + leftFeedforward);
    rightMaster.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Sets up the talons This method sets up the motor controllers.
   *
   * @since 1/30/2023
   */
  private void setupTalons() {
    // Inverts the right side of the drive train
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

  /**
   * Reset the encoders, sets their position to 0.
   *
   * @since 1/30/2023
   */
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param speed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   * @since 1/30/2023
   */
  public void arcadeDrive(double speed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Sets the talon mode to either brake or coast.
   *
   * @param mode The mode to set the talons to.
   * @since 1/30/2023
   */
  public void setTalonMode(NeutralMode mode) {
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
  }

  /**
   * Updates the odometry of the drive train. This method is called in the periodic method.
   *
   * @since 1/30/2023
   */
  public void updateOdometry() {
    odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
    field.setRobotPose(odometry.getPoseMeters());
  }

  /**
   * Gets the current roll of the robot.
   *
   * @return The current roll of the robot in degrees.
   * @since 1/30/2023
   */
  public double getRoll() {
    return navx.getRoll();
  }
}
