package bhs.devilbotz.commands.assist;

import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmToPosition;
import bhs.devilbotz.lib.GamePieceTypes;
import bhs.devilbotz.lib.ScoreLevels;
import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Move the arm to the desired scoring position
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.AutoScore
 */
public class PrepareForScoring extends SequentialCommandGroup {
  public PrepareForScoring(Arm arm, ScoreLevels level, GamePieceTypes gamePiece) {
    super();
    double targetArmPosition = ArmConstants.POSITION_TOP;
    // TODO: fill in arm position based on level/gamePiece
    switch (gamePiece) {
      case CUBE:
        {
          switch (level) {
            case LOW:
              targetArmPosition = ArmConstants.POSITION_BOTTOM;
              break;
            case MID:
              targetArmPosition = ArmConstants.POSITION_MIDDLE;
              break;
            case HIGH:
              targetArmPosition = ArmConstants.POSITION_TOP;
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
              targetArmPosition = ArmConstants.POSITION_BOTTOM;
              break;
            case MID:
              targetArmPosition = ArmConstants.POSITION_MIDDLE;
              break;
            case HIGH:
            default:
              targetArmPosition = ArmConstants.POSITION_TOP;
              break;
          }
        }
        break;
    }
    addCommands(CommandDebug.start());
    addCommands(new ArmToPosition(arm, targetArmPosition));
    addCommands(CommandDebug.end());
  }
}
