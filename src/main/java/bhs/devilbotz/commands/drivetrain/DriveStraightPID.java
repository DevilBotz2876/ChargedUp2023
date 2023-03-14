// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** This command is a PID controller that drives the robot straight to a set distance. */
public class DriveStraightPID extends CommandBase {
  protected DriveTrain drivetrain;
  private PIDController distancePid;
  private PIDController straightPid;
  private double distance;
  private double startAngle;
  private double startDistance;
  private final SlewRateLimiter speedSlewRateLimiter =
      new SlewRateLimiter(DriveConstants.SLEW_RATE_LIMITER * 2);
  private double maxSpeed = 0; // in meters/sec
  /**
   * The constructor for the Drive Straight PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraightPID(DriveTrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    distancePid =
        new PIDController(
            Robot.getDriveTrainConstant("DISTANCE_P").asDouble(),
            Robot.getDriveTrainConstant("DISTANCE_I").asDouble(),
            Robot.getDriveTrainConstant("DISTANCE_D").asDouble());
    straightPid =
        new PIDController(
            Robot.getDriveTrainConstant("STRAIGHT_P").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_I").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_D").asDouble());
    distancePid.setTolerance(Robot.getDriveTrainConstant("DISTANCE_PID_TOLERANCE").asDouble());
    straightPid.enableContinuousInput(0, 360);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace();
    startAngle = drivetrain.getYaw();
    startDistance = drivetrain.getAverageDistance();
    CommandDebug.trace("startAngle: " + startAngle + " distance: " + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output =
        distancePid.calculate(drivetrain.getAverageDistance() - startDistance, distance);
    double turnError = straightPid.calculate(drivetrain.getYaw(), startAngle);
    double speed = speedSlewRateLimiter.calculate(output);
    if (0 != maxSpeed) {
      speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
    }
    drivetrain.arcadeDrive(speed, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace(
        "endAngle: "
            + drivetrain.getYaw()
            + " distance: "
            + (drivetrain.getAverageDistance() - startDistance));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePid.atSetpoint();
  }

  /**
   * Sets the max speed for driving straight
   *
   * @param maxSpeed max speed in meters/second. 0 indicates no speed limit.
   */
  protected void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
  }
}
