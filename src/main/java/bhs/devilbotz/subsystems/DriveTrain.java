package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.Constants.SysIdConstants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.utils.ShuffleboardManager;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
      new WPI_TalonSRX(Robot.getDriveTrainConstant("MOTOR_LEFT_MASTER_CAN_ID").asInt());
  private static final WPI_TalonSRX rightMaster =
      new WPI_TalonSRX(Robot.getDriveTrainConstant("MOTOR_RIGHT_MASTER_CAN_ID").asInt());
  private static final WPI_TalonSRX leftFollower =
      new WPI_TalonSRX(Robot.getDriveTrainConstant("MOTOR_LEFT_FOLLOWER_CAN_ID").asInt());
  private static final WPI_TalonSRX rightFollower =
      new WPI_TalonSRX(Robot.getDriveTrainConstant("MOTOR_RIGHT_FOLLOWER_CAN_ID").asInt());

  // Defines the NAVX, which is a gyroscope that we use to track the robot's position.
  // It is attached to the robot via SPI (Serial Peripheral Interface).
  private static final AHRS navx = new AHRS(SPI.Port.kMXP);

  // Defines the PID controllers for the left and right sides of the drive train.
  // These are used to control the speed of the motors proportionally to the speed of the wheels.
  private final PIDController leftPIDController =
      new PIDController(
          Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_P").asDouble(),
          Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_I").asDouble(),
          Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_D").asDouble());
  private final PIDController rightPIDController =
      new PIDController(
          Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_P").asDouble(),
          Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_I").asDouble(),
          Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_D").asDouble());

  // Defines the kinematics of the drive train, which is used to calculate the speed of the wheels.
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Robot.getDriveTrainConstant("TRACK_WIDTH").asDouble());

  // Defines the odometry of the drive train, which is used to calculate the position of the robot.
  private final DifferentialDriveOdometry odometry;

  // Defines the feedforward of the drive train, which is used to calculate the voltage needed to
  // move the robot.
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Robot.getSysIdConstant("FEED_FORWARD_LINEAR_S").asDouble(),
          Robot.getSysIdConstant("FEED_FORWARD_LINEAR_V").asDouble(),
          Robot.getSysIdConstant("FEED_FORWARD_LINEAR_A").asDouble());
  private final SimpleMotorFeedforward leftFeedforward =
      new SimpleMotorFeedforward(
          Robot.getSysIdConstant("LEFT_FEED_FORWARD_LINEAR_S").asDouble(),
          Robot.getSysIdConstant("LEFT_FEED_FORWARD_LINEAR_V").asDouble(),
          Robot.getSysIdConstant("LEFT_FEED_FORWARD_LINEAR_A").asDouble());
  private final SimpleMotorFeedforward rightFeedforward =
      new SimpleMotorFeedforward(
          Robot.getSysIdConstant("RIGHT_FEED_FORWARD_LINEAR_S").asDouble(),
          Robot.getSysIdConstant("RIGHT_FEED_FORWARD_LINEAR_V").asDouble(),
          Robot.getSysIdConstant("RIGHT_FEED_FORWARD_LINEAR_A").asDouble());

  // Defines the field, which is used to display the robot's position on the field in Shuffleboard.
  private final Field2d field = new Field2d();

  // Object for simulated inputs into Talon.
  private static final TalonSRXSimCollection leftMasterSim = leftMaster.getSimCollection();
  private static final TalonSRXSimCollection rightMasterSim = rightMaster.getSimCollection();

  /**
   * The simulation model of our drivetrain
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html">Creating
   *     a Drivetrain Model</a>
   * @see <a
   *     href="https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L6">CTRE
   *     DifferentialDrive Simulation Example</a>
   * @since 1/31/2023
   */
  private DifferentialDrivetrainSim differentialDriveSim =
      new DifferentialDrivetrainSim(
          // Create a linear system from our identification gains.
          SysIdConstants.PLANT,
          DriveConstants.MOTOR_CONFIGURATION,
          Robot.getDriveTrainConstant("MOTOR_GEAR_RATIO").asDouble(),
          Robot.getDriveTrainConstant("TRACK_WIDTH").asDouble(),
          Robot.getDriveTrainConstant("WHEEL_RADIUS").asDouble(),

          // The standard deviations for measurement noise:
          // x and y:          0.001 m
          // heading:          0.001 rad
          // l and r velocity: 0.1   m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

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
  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations =
        positionMeters / (2 * Math.PI * Robot.getDriveTrainConstant("WHEEL_RADIUS").asDouble());
    double motorRotations =
        wheelRotations * Robot.getDriveTrainConstant("ENCODER_GEAR_RATIO").asDouble();
    int sensorCounts =
        (int) (motorRotations * Robot.getDriveTrainConstant("ENCODER_RESOLUTION").asInt());
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
  private int velocityToNativeUnits(double velocityMetersPerSecond) {
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
  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations =
        (double) sensorCounts / Robot.getDriveTrainConstant("ENCODER_RESOLUTION").asInt();
    double wheelRotations =
        motorRotations / Robot.getDriveTrainConstant("ENCODER_GEAR_RATIO").asDouble();
    double positionMeters =
        wheelRotations * (2 * Math.PI * Robot.getDriveTrainConstant("WHEEL_RADIUS").asDouble());
    return positionMeters;
  }

  /**
   * Helper function to convert Talon SRX sensor counts per 100ms to meters/second. Used for
   * Simulation.
   *
   * @param sensorCounts The robot's encoder count per 100ms
   * @return The robot's current velocity in meters per second
   * @see com.ctre.phoenix.motorcontrol.TalonSRXSimCollection#setQuadratureVelocity(int)
   * @see <a
   *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L223">CTRE
   *     Sample Code</a>
   * @since 1/31/2023
   */
  private double nativeUnitsToVelocityMetersPerSecond(double sensorCountsPer100ms) {
    return nativeUnitsToDistanceMeters(10 * sensorCountsPer100ms);
  }

  NetworkTableEntry xSimStart =
      NetworkTableInstance.getDefault().getTable("Simulation").getEntry("x_start_pose");
  NetworkTableEntry ySimStart =
      NetworkTableInstance.getDefault().getTable("Simulation").getEntry("y_start_pose");
  NetworkTableEntry rotSimStart =
      NetworkTableInstance.getDefault().getTable("Simulation").getEntry("rot_start_pose");
  NetworkTableEntry readSimStart =
      NetworkTableInstance.getDefault().getTable("Simulation").getEntry("read_start_pose");

  /**
   * DriveTrain constructor This constructor sets up the drive train.
   *
   * @since 1/30/2023
   */
  public DriveTrain() {
    // Sets the motor controllers to the correct mode & inverts the right side
    setupTalons();
    // Sets the initial position of the robot to (0, 0) and the initial angle to 0 degrees.
    resetEncoders();

    ShuffleboardManager.putField(field);

    // Defines the odometry of the drive train, which is used to calculate the position of the
    // robot.
    odometry =
        new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());

    xSimStart.setNumber(2.0);
    ySimStart.setNumber(2.0);
    rotSimStart.setNumber(0.0);
    readSimStart.setBoolean(false);

    SmartDashboard.putData("Left Velocity PID", leftPIDController);
    SmartDashboard.putData("Right Velocity PID", rightPIDController);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navx.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
    differentialDriveSim.setPose(pose);
  }

  /**
   * Resets the robot position to pose specified in network tables. Allows a user to choose robot
   * starting position by entering x, y, and rotation values via Shuffleboard or Smartdashboard.
   * User must toggle boolean to true to read from network tables. Otherwise the last position of
   * robot on field is used. A user can also enable Test mode and drag/drop robot around field to
   * any position.
   */
  public void resetRobotPosition() {
    Pose2d newPose;
    if (readSimStart.getBoolean(false)) {
      double x = xSimStart.getDouble(1.0);
      double y = ySimStart.getDouble(1.0);
      Rotation2d rot = new Rotation2d(rotSimStart.getDouble(0.0));
      newPose = new Pose2d(x, y, rot);
    } else {
      newPose = field.getRobotPose();
    }
    resetOdometry(newPose);
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
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /**
   * This method updates once per loop of the robot only in simulation mode. It is not run when
   * deployed to the physical robot.
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/device-sim.html">Device
   *     Simulation</a>
   * @since 1/30/2023
   */
  @Override
  public void simulationPeriodic() {
    /**
     * Simulate motors and integrated sensors
     *
     * @see <a
     *     href="https://github.com/crosstheroadelec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L144"</a>
     * @since 1/30/2023
     */

    // Pass the robot battery voltage to the simulated Talon SRXs
    leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());

    /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     *
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     *
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    differentialDriveSim.setInputs(
        leftMasterSim.getMotorOutputLeadVoltage(), -rightMasterSim.getMotorOutputLeadVoltage());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    differentialDriveSim.update(0.02);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     *
     * We also observe on our physical robot that a positive voltage
     * across the output leads results in a positive sensor velocity
     * for both the left and right motors, so we do not need to negate
     * the output any further.
     * If we had observed that a positive voltage results in a negative
     * sensor velocity, we would need to negate the output once more.
     */
    leftMasterSim.setQuadratureRawPosition(
        distanceToNativeUnits(differentialDriveSim.getLeftPositionMeters()));
    leftMasterSim.setQuadratureVelocity(
        velocityToNativeUnits(differentialDriveSim.getLeftVelocityMetersPerSecond()));
    rightMasterSim.setQuadratureRawPosition(
        distanceToNativeUnits(-differentialDriveSim.getRightPositionMeters()));
    rightMasterSim.setQuadratureVelocity(
        velocityToNativeUnits(-differentialDriveSim.getRightVelocityMetersPerSecond()));

    /**
     * Simulate navX
     *
     * @see <a href="https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/"</a>
     */
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-differentialDriveSim.getHeading().getDegrees());
  }

  /**
   * Get the left encoder distance.
   *
   * @return The left encoder distance in meters.
   * @see #getRightDistance()
   * @since 1/30/2023
   */
  public double getLeftDistance() {
    return nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition());
  }

  /**
   * Get the right encoder distance.
   *
   * @return The right encoder distance in meters.
   * @see #getLeftDistance()
   * @since 1/30/2023
   */
  public double getRightDistance() {
    return nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition());
  }

  /**
   * Get the average distance of both encoders
   *
   * @return the average distance of both encoders in meters
   */
  public double getAverageDistance() {
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

  /**
   * Sets the desired wheel speeds using the PID controllers.
   *
   * @param speeds The desired wheel speeds in meters per second.
   * @since 1/30/2023
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculates the desired voltages for the left and right sides of the drive train.
    final double leftFeedforward = this.leftFeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = this.rightFeedforward.calculate(speeds.rightMetersPerSecond);

    // Calculates the PID output for the left and right sides of the drive train.
    final double leftOutput =
        leftPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    // Sets the motor controller speeds.
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
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
    if (RobotBase.isSimulation()) {
      // TODO: Understand why the sensor phase needs to be swapped for simulation
      rightMaster.setSensorPhase(false);
      leftMaster.setSensorPhase(false);
    } else {
      rightMaster.setSensorPhase(true);
      leftMaster.setSensorPhase(true);
    }
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
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // Sets the motor controller speeds.
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
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
  private void updateOdometry() {
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
  /**
   * Gets the current yaw of the robot. It is the value of the gyro when it turns left and right
   *
   * @return The current yaw of the robot in degrees.
   */
  public double getYaw() {
    return navx.getYaw();
  }

  /**
   * Uses ramsete controller to follow the specified trajectory
   *
   * @param traj Requested trajectory
   * @param isFirstPath Set to true if this is the first path being run in autonomous in order to
   *     reset odometry before starting
   * @return A sequential command that when executed, moves the robot along the specified trajectory
   * @see <a
   *     href=https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppramsetecommand>PathPlanner
   *     Example</a>
   */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                this.resetOdometry(traj.getInitialPose());
              }
            }),
        new PPRamseteCommand(
            traj,
            this::getPose, // Pose supplier
            new RamseteController(),
            feedforward,
            this.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(
                Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_P").asDouble(),
                Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_I").asDouble(),
                Robot.getSysIdConstant("LEFT_FEED_BACK_VELOCITY_D").asDouble()),
            new PIDController(
                Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_P").asDouble(),
                Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_I").asDouble(),
                Robot.getSysIdConstant("RIGHT_FEED_BACK_VELOCITY_D").asDouble()),
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            this // Requires this drive subsystem
            ),
        new InstantCommand(
            () -> {
              this.tankDriveVolts(0, 0);
            }));
  }
}
