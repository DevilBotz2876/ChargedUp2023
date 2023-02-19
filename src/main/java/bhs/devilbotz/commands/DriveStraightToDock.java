// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands;

import bhs.devilbotz.Robot;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command is a PID controller that drives the robot straight to a set distance up a hill to
 * dock with the charging port.
 */
public class DriveStraightToDock extends CommandBase {
  private DriveTrain drivetrain;
  private PIDController distancePid;
  private PIDController straightPid;
  private double distance;
  private double startAngle;
  private final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(1);

  /**
   * The constructor for the Drive Straight To Dock PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraightToDock(DriveTrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    distancePid =
        // new PIDController(Constants.DISTANCE_P, Constants.DISTANCE_I, Constants.DISTANCE_D);
        new PIDController(
            Robot.getDriveTrainConstant("DISTANCE_P").asDouble(),
            Robot.getDriveTrainConstant("DISTANCE_I").asDouble(),
            Robot.getDriveTrainConstant("DISTANCE_D").asDouble());
    straightPid =
        // new PIDController(Constants.STRAIGHT_P, Constants.STRAIGHT_I, Constants.STRAIGHT_D);
        new PIDController(
            Robot.getDriveTrainConstant("STRAIGHT_P").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_I").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_D").asDouble());
    startAngle = drivetrain.getYaw();
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveStraightToDock start");
    drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // distance_pid.setTolerance(1);
    double output = distancePid.calculate(drivetrain.getAverageDistance(), distance);
    double turnError = straightPid.calculate(drivetrain.getYaw(), startAngle);
    drivetrain.arcadeDrive(speedSlewRateLimiter.calculate(output), -turnError);

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
    System.out.println("DriveStraightToDock Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return distance_pid.atSetpoint();
    // double roll = Math.abs(drivetrain.getRoll());
    // if (roll > 7){
    //  roll = 0;
    // }
    if (Math.abs(distancePid.getPositionError()) < 0.1) {
      return true;
    }
    return false;
  }
}
