// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Robot;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final DriveTrain drive;
  // private double levelAngle;
  private final PIDController balancePid;
  Timer timer = new Timer();

  /**
   * The constructor for the balance PID command.
   *
   * @param drive The drive train subsystem.
   * @param balancePid The external PID for controlling balance
   */
  public BalancePID(DriveTrain drive, PIDController balancePid) {
    this.drive = drive;
    this.balancePid = balancePid;
    addRequirements(drive);
  }

  /**
   * The constructor for the balance PID command.
   *
   * @param drive The drive train subsystem.
   */
  public BalancePID(DriveTrain drive) {
    this(
        drive,
        new PIDController(
            Robot.getDriveTrainConstant("BALANCE_P").asDouble(),
            Robot.getDriveTrainConstant("BALANCE_I").asDouble(),
            Robot.getDriveTrainConstant("BALANCE_D").asDouble()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("BalancePID start");
    drive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = balancePid.calculate(drive.getRoll(), 0);
    double output = MathUtil.clamp(error, -0.25, 0.25);
    if (Math.abs(drive.getRoll()) < Constants.BALANCE_PID_TOLERANCE) {
      output = 0;
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
    drive.arcadeDrive(-output, 0);

    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("angle", drive.getRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    System.out.println("BalancePID Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(0.5)) {
      return true;
    }
    return false;
  }
}
