package bhs.devilbotz.subsystems.drive;

import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
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
      new DifferentialDriveKinematics(Robot.getDriveConstant("TRACK_WIDTH").asDouble());

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

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();

  public Drive(DriveIO io) {
    this.io = io;
    odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(inputs.yawPositionRad),
            inputs.leftDistanceMeters,
            inputs.rightDistanceMeters);

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.getInstance().processInputs("Drive", inputs);

    updateOdometry();
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Updates the odometry of the drive train. This method is called in the periodic method. */
  private void updateOdometry() {
    odometry.update(
        new Rotation2d(inputs.yawPositionRad),
        inputs.leftDistanceMeters,
        inputs.rightDistanceMeters);
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        new Rotation2d(inputs.yawPositionRad),
        inputs.leftDistanceMeters,
        inputs.rightDistanceMeters,
        pose);
  }

  public void resetEncoders() {
    io.resetEncoders();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        inputs.leftVelocityMetersPerSec, inputs.rightVelocityMetersPerSec);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculates the desired voltages for the left and right sides of the drive train.
    final double leftFeedforward = this.leftFeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = this.rightFeedforward.calculate(speeds.rightMetersPerSecond);

    // Calculates the PID output for the left and right sides of the drive train.
    final double leftOutput =
        leftPIDController.calculate(inputs.leftVelocityMetersPerSec, speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(inputs.rightVelocityMetersPerSec, speeds.rightMetersPerSecond);

    // Sets the motor controller speeds.
    tankDriveVolts(leftFeedforward + leftOutput, rightFeedforward + rightOutput);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
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
    System.out.println(wheelSpeeds);
    setSpeeds(wheelSpeeds);
  }

  /**
   * Gets the current roll of the robot.
   *
   * @return The current roll of the robot in degrees.
   * @since 1/30/2023
   */
  public double getRoll() {
    double roll = inputs.rollPositionDeg;
    if (Robot.checkCapability("hasInvertedRoll")) {
      roll = -roll;
    }
    return roll;
  }

  /**
   * Gets the current yaw of the robot. It is the value of the gyro when it turns left and right
   *
   * @return The current yaw of the robot in degrees.
   */
  public double getYaw() {
    return inputs.yawPositionDeg;
  }

  public double getAverageDistance() {
    return inputs.averageDistanceMeters;
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
  public Command followTrajectoryCommand(
      PathPlannerTrajectory traj, boolean isFirstPath, boolean stopAtEnd) {
    final PathPlannerTrajectory translatedTraj =
        PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
    return new SequentialCommandGroup(
        CommandDebug.message("followTrajectoryCommand:Start"),
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                this.resetOdometry(translatedTraj.getInitialPose());
              }
            }),
        new PPRamseteCommand(
            translatedTraj,
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
            false, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            this // Requires this drive subsystem
            ),
        new InstantCommand(
            () -> {
              if (stopAtEnd) {
                this.tankDriveVolts(0, 0);
              }
            }),
        CommandDebug.message("followTrajectoryCommand:End"));
  }

  public Command stopCommand() {
    return new InstantCommand(
        () -> {
          this.tankDriveVolts(0, 0);
        },
        this);
  }
}
