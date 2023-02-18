// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command moves the arm down.
 *
 * @since 1/25/2023
 * @author joshuamanoj
 */
public class ArmMoveDistance extends CommandBase {
  private final Arm arm;
  private final double distance;
  private double start, end;

  /**
   * The constructor
   *
   * @param arm The arm subsystem.
   */
  public ArmMoveDistance(Arm arm, double distance) {
    this.arm = arm;
    this.distance = distance;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = arm.getPosition();
    end = start + distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (distance > 0) {
      arm.up();
    } else if (distance < 0) {
      arm.down();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double diff = arm.getPosition() - end;
    return Math.abs(diff) < 2;
  }
}
