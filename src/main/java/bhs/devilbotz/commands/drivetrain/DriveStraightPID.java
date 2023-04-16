// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** This command is a PID controller that drives the robot straight to a set distance. */
public class DriveStraightPID extends CommandBase {
  protected Drive drive;
  private PIDController distancePid;
  private PIDController straightPid;
  private double distance;
  private double startAngle;
  private double startDistance;
  private final SlewRateLimiter speedSlewRateLimiter =
      new SlewRateLimiter(DriveConstants.SLEW_RATE_LIMITER / 2);
  private double maxSpeed = 0; // in meters/sec
  /**
   * The constructor for the Drive Straight PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraightPID(Drive drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    distancePid =
        new PIDController(
            Robot.getDriveConstant("DISTANCE_P").asDouble(),
            Robot.getDriveConstant("DISTANCE_I").asDouble(),
            Robot.getDriveConstant("DISTANCE_D").asDouble());
    straightPid =
        new PIDController(
            Robot.getDriveConstant("STRAIGHT_P").asDouble(),
            Robot.getDriveConstant("STRAIGHT_I").asDouble(),
            Robot.getDriveConstant("STRAIGHT_D").asDouble());
    distancePid.setTolerance(Robot.getDriveConstant("DISTANCE_PID_TOLERANCE").asDouble());
    straightPid.enableContinuousInput(0, 360);
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * The constructor for the Drive Straight PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   * @param maxSpeed The max speed the robot should travel
   */
  public DriveStraightPID(Drive drive, double distance, double maxSpeed) {
    this(drive, distance);

    this.maxSpeed = maxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace();
    startAngle = drive.getYaw();
    startDistance = drive.getAverageDistance();
    CommandDebug.trace("startAngle: " + startAngle + " distance: " + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = distancePid.calculate(drive.getAverageDistance() - startDistance, distance);
    double turnError = straightPid.calculate(drive.getYaw(), startAngle);
    double speed = speedSlewRateLimiter.calculate(output);
    if (0 != maxSpeed) {
      speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
    }
    drive.arcadeDrive(speed, turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace(
        "endAngle: "
            + drive.getYaw()
            + " distance: "
            + (drive.getAverageDistance() - startDistance));
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
