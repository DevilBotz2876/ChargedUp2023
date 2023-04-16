// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command is a PID controller that balances the robot on the charge platform.
 *
 * @since 1/21/2023
 * @author firydragon57
 */
public class BalancePID extends CommandBase {
  /** Creates a new BalancePID. */
  // TODO: Add a feedforward term to the PID controller and tune pid
  private final Drive drive;
  // private double levelAngle;
  private final PIDController balancePid;
  Timer timer = new Timer();
  private double targetAngle;
  private boolean targetAngleValid = false;
  private final PIDController straightPid;

  /**
   * The constructor for the balance PID command.
   *
   * @param drive The drive train subsystem.
   * @param balancePid The external PID for controlling balance
   */
  public BalancePID(Drive drive, PIDController balancePid, PIDController straightPid) {
    this.drive = drive;
    this.balancePid = balancePid;
    this.straightPid = straightPid;

    straightPid.enableContinuousInput(0, 360);
    addRequirements(drive);
  }

  /**
   * The constructor for the balance PID command.
   *
   * @param drive The drive train subsystem.
   */
  public BalancePID(Drive drive) {
    this(
        drive,
        new PIDController(
            Robot.getDriveConstant("BALANCE_P").asDouble(),
            Robot.getDriveConstant("BALANCE_I").asDouble(),
            Robot.getDriveConstant("BALANCE_D").asDouble()),
        null);
  }

  public BalancePID(Drive drive, double targetAngle) {
    this(
        drive,
        new PIDController(
            Robot.getDriveConstant("BALANCE_P").asDouble(),
            Robot.getDriveConstant("BALANCE_I").asDouble(),
            Robot.getDriveConstant("BALANCE_D").asDouble()),
        new PIDController(
            Robot.getDriveConstant("STRAIGHT_P").asDouble(),
            Robot.getDriveConstant("STRAIGHT_I").asDouble(),
            Robot.getDriveConstant("STRAIGHT_D").asDouble()));

    this.targetAngle = targetAngle;
    this.targetAngleValid = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandDebug.trace("startRoll: " + drive.getRoll() + " targetAngle: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = balancePid.calculate(drive.getRoll(), 0);
    double maxSpeed = Robot.getDriveConstant("BALANCE_MAX_SPEED").asDouble();
    double output = MathUtil.clamp(error, -maxSpeed, maxSpeed);
    double turnError = 0;

    if (true == targetAngleValid) {
      turnError = straightPid.calculate(drive.getYaw(), targetAngle);
    }

    if (Math.abs(drive.getRoll()) < Constants.BALANCE_PID_TOLERANCE) {
      /* When the robot is within the balance tolerance we:
       *  1) stop the robot from moving
       *  2) start a timer to measure how long we've been balanced
       */
      output = 0;
      timer.start();
    } else {
      /* When the robot is *not* within the balance tolerance we:
       *  1) stop and reset the timer
       */
      timer.stop();
      timer.reset();
    }
    drive.arcadeDrive(-output, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace("endRoll: " + drive.getRoll() + " endAngle: " + drive.getYaw());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Empirically, if we've been balanced for at least the min duration, we assume we are done
    if (timer.hasElapsed(Robot.getDriveConstant("BALANCE_MIN_DURATION").asDouble())) {
      return true;
    }
    return false;
  }
}
