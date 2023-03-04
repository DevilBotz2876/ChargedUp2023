package bhs.devilbotz.commands.assist;

import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.lib.ScoreLevels;
import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepareForScoring extends SequentialCommandGroup {
  private final double CONE_POSITION_TOP = 558;
  private final double CONE_POSITION_MIDDLE = 468;
  private final double CONE_POSITION_BOTTOM = 258;
  private final double CONE_POSITION_PORTAL = 465;

  public PrepareForScoring(Arm arm, ScoreLevels level, GamePieceTypes gamePiece) {
    super();
    double targetArmPosition;
    // TODO: fill in arm position based on level/gamePiece
    switch (gamePiece) {
      case CUBE:
        {
          switch (level) {
            case LOW:
              targetArmPosition = CONE_POSITION_BOTTOM;
              break;
            case MID:
              targetArmPosition = CONE_POSITION_MIDDLE;
              break;
            case HIGH:
              targetArmPosition = CONE_POSITION_TOP;
            default:
              break;
          }
        }
        break;

      case CONE:
      default:
        {
          switch (level) {
            case LOW:
              targetArmPosition = CONE_POSITION_BOTTOM;
              break;
            case MID:
              targetArmPosition = CONE_POSITION_MIDDLE;
              break;
            case HIGH:
            default:
              targetArmPosition = CONE_POSITION_TOP;
              break;
          }
        }
        break;
    }
    // TODO: addCommands(new ArmToPosition(arm, targetArmPoisition));
  }
}
