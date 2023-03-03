// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Robot;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** This command is a PID controller that drives the robot straight to a set distance. */
public class DriveStraightPID extends CommandBase {
  protected DriveTrain drivetrain;
  private PIDController distancePid;
  private PIDController straightPid;
  private double distance;
  private double startAngle;
  private double startDistance;
  private final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(1);
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

    SmartDashboard.putData("Distance PID", distancePid);
    SmartDashboard.putData("Straight PID", straightPid);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveStraightPID start");
    drivetrain.arcadeDrive(0, 0);
    startAngle = drivetrain.getYaw();
    startDistance = drivetrain.getAverageDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = distancePid.calculate(drivetrain.getAverageDistance()-startDistance, distance);
    double turnError = straightPid.calculate(drivetrain.getYaw(), startAngle);
    double speed = speedSlewRateLimiter.calculate(output);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Speed", speed);
    if (0 != maxSpeed) {
      speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
    }
    drivetrain.arcadeDrive(speed, -turnError);

    SmartDashboard.putNumber("Distance output", output);
    SmartDashboard.putNumber("Position Tolerance", distancePid.getPositionTolerance());
    SmartDashboard.putBoolean("at Setpoint", distancePid.atSetpoint());
    SmartDashboard.putNumber("Position Error", distancePid.getPositionError());
    SmartDashboard.putNumber("Distance", drivetrain.getAverageDistance());

    SmartDashboard.putNumber("Turn output", turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    System.out.println("DriveStraightPID Finished");
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
