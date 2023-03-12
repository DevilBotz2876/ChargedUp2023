// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command is a PID controller that drives the robot straight to a set distance up a hill to
 * dock with the charging port.
 */
public class DriveStraightToDock extends CommandBase {
  private enum DockState {
    ON_GROUND,
    ON_RAMP,
    LEVELING_OFF
  }

  private DockState currentState = DockState.ON_GROUND;
  protected DriveTrain drivetrain;
  private PIDController straightPid;
  private final double maxDistance;
  private double startAngle;
  private double startDistance;

  /**
   * The constructor for the Drive Straight To Dock PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param maxDistance The MAX distance (in meters) the robot needs to cover. We end when the
   *     distance is reached OR we've detected that we are physically on the dock
   */
  public DriveStraightToDock(DriveTrain drivetrain, double maxDistance) {
    this.drivetrain = drivetrain;
    this.maxDistance = maxDistance;
    straightPid =
        new PIDController(
            Robot.getDriveTrainConstant("STRAIGHT_P").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_I").asDouble(),
            Robot.getDriveTrainConstant("STRAIGHT_D").asDouble());
    straightPid.enableContinuousInput(0, 360);
    addRequirements(drivetrain);
  }

  int onRampCount = 0;
  double previousRoll;
  int levelingRampCount = 0;

  @Override
  public void initialize() {
    CommandDebug.trace();
    startAngle = drivetrain.getYaw();
    startDistance = drivetrain.getAverageDistance();
    CommandDebug.trace(
        "startRoll: "
            + drivetrain.getRoll()
            + "startAngle: "
            + startAngle
            + " maxDistance: "
            + maxDistance);
  }

  @Override
  public void execute() {
    double currentRoll = drivetrain.getRoll();
    double speed;

    CommandDebug.trace("currentRoll: " + currentRoll);
    
    switch (currentState) {
      case ON_GROUND:
        /* We start in the ON_GROUND state. We see if the currentRoll is within the expected ramp angle.
         * Empirically, the roll is between min/max when on the ramp.
         * When approaching the ramp, we cap the max speed to prevent crashing into the ramp
         */
        speed = Robot.getDriveTrainConstant("DOCK_MAX_SPEED_ON_GROUND").asDouble(0.75);

        // We use the absolute value of the current roll so that we can approach the ram either
        // going forward or backwards.
        if ((Math.abs(currentRoll) > Robot.getDriveTrainConstant("DOCK_MIN_RAMP_ROLL").asDouble(10))
            && (Math.abs(currentRoll)
                < Robot.getDriveTrainConstant("DOCK_MAX_RAMP_ROLL").asDouble(15))) {
          if (0 == onRampCount) {
            CommandDebug.message("On Ramp?");
          }
          onRampCount++;
        } else {
          if (0 != onRampCount) {
            CommandDebug.message("No, NOT on Ramp...");
          }
          onRampCount = 0;
        }

        /* If we've been on the ramp long enough (roll is within expected window), we assume we are on the ramp and transition states */
        if (onRampCount > Robot.getDriveTrainConstant("DOCK_MIN_ON_RAMP_COUNT").asInt(10)) {
          CommandDebug.message("Yes, On Ramp! " + onRampCount);
          currentState = DockState.ON_RAMP;
        }
        break;

      case ON_RAMP:
        /* When we think we are on the ramp, we reduce the maxSpeed so that we don't go so fast that we overshoot and cause the
         * ramp to teeter quickly to the other side
         * While the roll is decreasing and the currentRoll is less than the empirical ramp roll we assume we are leveling off.
         */
        speed = Robot.getDriveTrainConstant("DOCK_MAX_SPEED_ON_RAMP").asDouble(0.50);
        double deltaRoll = currentRoll - previousRoll;
        if ((deltaRoll < 0)
            && (currentRoll < Robot.getDriveTrainConstant("DOCK_MIN_RAMP_ROLL").asDouble(10))) {
          if (0 == levelingRampCount) {
            CommandDebug.message("Leveling Off?");
          }
          levelingRampCount++;
        } else {
          if (0 != levelingRampCount) {
            CommandDebug.message("No, NOT leveling off...");
          }
          levelingRampCount = 0;
        }

        /* If we've been leveling off long enough, we assume we are almost balanced */
        if (levelingRampCount > Robot.getDriveTrainConstant("DOCK_MIN_LEVELING_COUNT").asInt(2)) {
          CommandDebug.message("Yes, leveling off! " + levelingRampCount);
          currentState = DockState.LEVELING_OFF;
        }
        break;

      case LEVELING_OFF:
      default:
        speed = 0;
        break;
    }

    /* We save the current roll so we can calculate the deltaRoll next time */
    previousRoll = currentRoll;

    double turnError = straightPid.calculate(drivetrain.getYaw(), startAngle);
    drivetrain.arcadeDrive(speed, -turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace(
        "endRoll: "
            + drivetrain.getRoll()
            + "endAngle: "
            + drivetrain.getYaw()
            + " distance: "
            + (drivetrain.getAverageDistance() - startDistance));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = drivetrain.getAverageDistance() - startDistance;
    /* We are finished when we are in the LEVELING_OFF state.
     * As a backup, we also assume we are finished when the total distance has been traveled
     */
    return ((DockState.LEVELING_OFF == currentState)
        || (Math.abs(distanceTravelled) >= Math.abs(maxDistance)));
  }
}
