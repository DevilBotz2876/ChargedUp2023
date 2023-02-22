package bhs.devilbotz.lib;

/**
 * Autonomous modes enum for the autonomous chooser on the dashboard.
 *
 * @see <a href=https://github.com/BHSRobotix/ChargedUp2023#desired-autonomous-routines>Desired
 *     Autonomous Routines</a>
 * @since 1/18/2023
 */
public enum AutonomousModes {
  /**
   * Don't do anything in autonomous mode. Sit still and let the other alliance members do the work.
   */
  SIT_STILL,
  /** Drive straight forwards towards the middle of the field */
  MOBILITY,
  /** Score preloaded game piece and then perform {@link #MOBILITY} */
  SCORE_AND_MOBILITY,
  /** Drive straight onto charge station and balance */
  DOCK_AND_ENGAGE,
  /** Perform {@link #MOBILITY} and then {@link #DOCK_AND_ENGAGE} */
  MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE,
  /** Perform {@link #MOBILITY} and then {@link #DOCK_AND_ENGAGE} */
  MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE,
  /** Score preloaded game piece and then perform {@link #DOCK_AND_ENGAGE} */
  SCORE_DOCK_AND_ENGAGE,
  /** Perform {@link #SCORE_AND_MOBILITY} and then {@link #DOCK_AND_ENGAGE} */
  SCORE_MOBILITY_DOCK_ENGAGE,
  /**
   * Perform {@link #SCORE_AND_MOBILITY}, pick up a game piece at center field, and then {@link
   * #DOCK_AND_ENGAGE}
   */
  SCORE_MOBILITY_PICK_DOCK_ENGAGE,
  /** Placeholder For Testing New Autonomous Routines */
  TEST
}
