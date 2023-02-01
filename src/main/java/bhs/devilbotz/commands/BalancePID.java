// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands;

import bhs.devilbotz.Constants;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  private final PIDController pid;

  /**
   * The constructor for the balance PID command.
   * @param drive The drive train subsystem.
   */
  public BalancePID(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    pid = new PIDController(Constants.BALANCE_P, Constants.BALANCE_I, Constants.BALANCE_D);
    addRequirements(drive);
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
    double error = pid.calculate(drive.getRoll(), 0);
    double output = MathUtil.clamp(error, -0.5, 0.5);

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
    return false;
  }
}
