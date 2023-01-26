// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancePID extends CommandBase {
  /** Creates a new BalancePID. */
  private final DriveTrain m_drive;
  // private double levelAngle;
  PIDController pid;

  public BalancePID(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    pid = new PIDController(0.07, 0, 0.01);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("BalancePID start");
    m_drive.arcadeDrive(0, 0);
    // m_drive.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = pid.calculate(m_drive.getRoll(), 0);
    double output = MathUtil.clamp(error, -0.5, 0.5);

    m_drive.arcadeDrive(-output, 0);

    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("angle", m_drive.getRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    System.out.println("BalancePID Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(pid.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}
