package bhs.devilbotz.lib;

/**
 * Autonomous modes enum for the autonomous chooser on the dashboard.
 *
 * @since 1/18/2023
 */
public enum AutonomousModes {
  SIT_STILL,
  MOBILITY,
  SCORE_AND_MOBILITY,
  DOCK_AND_ENGAGE,
  MOBILITY_DOCK_AND_ENGAGE,
  SCORE_DOCK_AND_ENGAGE,
  SCORE_MOBILITY_DOCK_ENGAGE,
  SCORE_MOBILITY_PICK_DOCK_ENGAGE,

  /** Backup short moves the robot backwards a short distance and then stops. */
  BACKUP_SHORT,
  /**
   * Backup and balance moves the robot backwards a short distance and then balances on the
   * platform.
   */
  BACKUP_AND_BALANCE,
  /** Backup far moves the robot backwards a long distance and then stops. */
  BACKUP_FAR,
  /** Balance does NOT move the robot and then balances on the platform. */
  BALANCE,
  /** Drive Distance goes forward one meter */
  DRIVE_DISTANCE,
  /** Drive Distance goes forward one meter using PID for */
  DRIVE_DISTANCE_PID,
  /** Drive Distance goes forward one meter using PID */
  DRIVE_STRAIGHT_DISTANCE_PID,
}
