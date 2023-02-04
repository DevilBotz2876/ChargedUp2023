// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSetDistancePID extends CommandBase {
  private DriveTrain drivetrain;
  private PIDController distance_pid;
  private double distance;

  public DriveSetDistancePID(DriveTrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    distance_pid = new PIDController(0.25, 0, 0.1);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveStraightPID start");
    drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // distance_pid.setTolerance(1);
    double output = distance_pid.calculate(drivetrain.getAverageDistance(), distance);
    drivetrain.arcadeDrive(output, 0);

    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("Position Tolerance", distance_pid.getPositionTolerance());
    SmartDashboard.putBoolean("at Setpoint", distance_pid.atSetpoint());
    SmartDashboard.putNumber("Position Error", distance_pid.getPositionError());
    SmartDashboard.putNumber("Distance", drivetrain.getAverageDistance());
    SmartDashboard.putNumber("Velocity Error", distance_pid.getVelocityError());
    SmartDashboard.putNumber("Velocity Tolerance", distance_pid.getVelocityTolerance());
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
    // return distance_pid.atSetpoint();
    if (Math.abs(distance_pid.getPositionError()) < 0.001) {
      return true;
    }
    return false;
  }
}
