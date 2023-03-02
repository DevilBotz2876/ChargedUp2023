// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.arm;

import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command moves the arm to middle scoring position. Middle is determined by encoder position.
 */
public class ArmToMiddle extends CommandBase {
  private final Arm arm;

  /**
   * The constructor for the arm down command.
   *
   * @param arm The arm subsystem.
   */
  public ArmToMiddle(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.aboveMiddle()) {
      arm.down();

      if (arm.isBottomLimit()) {
        arm.up();
      }
    } else if (arm.belowMiddle()) {
      arm.up();

      if (arm.isTopLimit()) {
        arm.down();
      }
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
    // Stop command in case the end position is out of range or encoder is broken or giving
    // incorrect readings
    
    return arm.atMiddle();
  }
}
