// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraight extends CommandBase {
  private DriveTrain drivetrain;
  private double distance;
  /**
   * The constructor for the Drive Straight command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraight(DriveTrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;

    addRequirements(drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(1, 0);
  }

  // // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double l = drivetrain.getLeftDistance();
    double r = drivetrain.getRightDistance();
    if (l < distance && r < distance) {
      return false;
    }
    return true;
  }
}
