// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.auto;

import bhs.devilbotz.subsystems.DriveTrain;

/**
 * This command is a PID controller that drives the robot straight to a set distance up a hill to
 * dock with the charging port.
 */
public class DriveStraightToDock extends DriveStraightPID {

  /**
   * The constructor for the Drive Straight To Dock PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraightToDock(DriveTrain drivetrain, double distance) {
    super(drivetrain, distance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
    // return distance_pid.atSetpoint();
    // double roll = Math.abs(drivetrain.getRoll());
    // if (roll > 7){
    //  roll = 0;
    // }
    //    if (Math.abs(distancePid.getPositionError()) < 0.1) {
    //      return true;
    //    }
    //    return false;
  }
}
