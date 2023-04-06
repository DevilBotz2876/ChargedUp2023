// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.commands.drivetrain;

import bhs.devilbotz.Robot;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.EnumMap;

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
  private DriveTrain drivetrain;
  private PIDController straightPid;
  private final double maxDistance;
  private double targetAngle;
  private boolean targetAngleValid = false;
  private double startDistance;
  private Timer timer = new Timer();
  EnumMap<DockState, Double> speeds = new EnumMap<>(DockState.class);

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

    speeds.put(
        DockState.ON_GROUND,
        Robot.getDriveTrainConstant("DOCK_MAX_SPEED_ON_GROUND")
            .asDouble(0.75)); // We drive fastest when approaching the ramp.
    speeds.put(
        DockState.ON_RAMP,
        Robot.getDriveTrainConstant("DOCK_MAX_SPEED_ON_RAMP")
            .asDouble(0.50)); // We then slow down when on the ramp
    speeds.put(DockState.LEVELING_OFF, 0.0); // We stop when we are leveling off

    addRequirements(drivetrain);
  }

  /**
   * The constructor for the Drive Straight To Dock PID command to specify the target angle.
   *
   * @param drivetrain The drive train subsystem.
   * @param maxDistance The MAX distance (in meters) the robot needs to cover. We end when the
   *     distance is reached OR we've detected that we are physically on the dock
   * @param targetAngle The desired target angle
   */
  public DriveStraightToDock(DriveTrain drivetrain, double maxDistance, double targetAngle) {
    this(drivetrain, maxDistance);

    this.targetAngle = targetAngle;
    this.targetAngleValid = true;
  }

  double previousRoll;

  @Override
  public void initialize() {
    CommandDebug.trace();
    if (false == targetAngleValid) {
      targetAngle = drivetrain.getYaw();
      targetAngleValid = true;
    }
    startDistance = drivetrain.getAverageDistance();
    CommandDebug.trace(
        "startRoll: "
            + drivetrain.getRoll()
            + " targetAngle: "
            + targetAngle
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
         */
        // We use the absolute value of the current roll so that we can approach the ramp either
        // going forward or backwards.
        if ((Math.abs(currentRoll) > Robot.getDriveTrainConstant("DOCK_RAMP_ROLL_MIN").asDouble(10))
            // To minimize breakage of working routines, ignore the upper roll value when driving
            // forward)
            && ((maxDistance < 0)
                || (Math.abs(currentRoll)
                    < Robot.getDriveTrainConstant("DOCK_RAMP_ROLL_MAX").asDouble(15)))) {
          // Looks like the roll reading is within range of being on the ramp, so start the timer if
          // it already hasn't been started
          if (0 == timer.get()) {
            CommandDebug.message("On Ramp?");
          }
          timer.start();
        } else {
          // We don't seem to be on the ramp, so reset the timer
          if (0 != timer.get()) {
            CommandDebug.message("No, NOT on Ramp...");
          }
          timer.stop();
          timer.reset();
        }

        /* If we've been on the ramp long enough (roll is within expected window), we assume we are on the ramp and transition states */
        if (timer.hasElapsed(
            Robot.getDriveTrainConstant("DOCK_ON_RAMP_DURATION_MIN").asDouble(0.2))) {
          CommandDebug.message("Yes, On Ramp!");
          // We skip ON_RAMP and go straight to LEVELING_OFF because the BalancePID can take over
          // once we are on the ramp
          currentState = DockState.LEVELING_OFF;
          timer.stop();
          timer.reset();
        }
        break;

      case ON_RAMP:
        /* When we think we are on the ramp, we reduce the maxSpeed so that we don't go so fast that we overshoot and cause the
         * ramp to teeter quickly to the other side
         * While the roll is decreasing and the currentRoll is less than the empirical ramp roll we assume we are leveling off.
         */
        double deltaRoll = currentRoll - previousRoll;
        if ((deltaRoll < 0)
            && (currentRoll < Robot.getDriveTrainConstant("DOCK_LEVELING_ROLL_MIN").asDouble(10))) {
          // Looks like the roll reading is within range of leveling off, so start the timer if it
          // already hasn't been started
          if (0 == timer.get()) {
            CommandDebug.message("Leveling Off?");
            timer.start();
          }
        } else {
          // We don't seem to be on the leveling off, so reset the timer
          if (0 != timer.get()) {
            CommandDebug.message("No, NOT leveling off...");
          }
          timer.stop();
          timer.reset();
        }

        /* If we've been leveling off long enough, we assume we are almost balanced */
        if (timer.hasElapsed(
            Robot.getDriveTrainConstant("DOCK_LEVELING_OFF_DURATION_MIN").asDouble(0.2))) {
          CommandDebug.message("Yes, leveling off!");
          currentState = DockState.LEVELING_OFF;
          timer.stop();
          timer.reset();
        }
        break;

      case LEVELING_OFF:
      default:
        break;
    }

    // We use a PID to determine how much to turn to maintain the same starting angle
    double turnError = straightPid.calculate(drivetrain.getYaw(), targetAngle);

    // Set the speed based on the currentState
    speed = speeds.get(currentState);

    // Drive backwards if maxDistance is negative
    if (maxDistance < 0) {
      speed = -speed;
    }

    drivetrain.arcadeDrive(speed, -turnError);

    /* We save the current roll so we can calculate the deltaRoll next time */
    previousRoll = currentRoll;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandDebug.trace(
        "endRoll: "
            + drivetrain.getRoll()
            + " endAngle: "
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
