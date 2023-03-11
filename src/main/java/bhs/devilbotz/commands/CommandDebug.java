package bhs.devilbotz.commands;

import bhs.devilbotz.Constants.DebugConstants;
import bhs.devilbotz.utils.Debug;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandDebug {
  static final Debug debug = new Debug("Command");

  public static void trace() {
    //    debug.trace();
    if (DebugConstants.enableArmMessages) {
      System.out.println(debug.getTraceString());
    }
  }

  public static void trace(String message) {
    if (DebugConstants.enableArmMessages) {
      System.out.println(debug.getTraceString() + ":" + message);
    }
  }

  public static void println(String message) {
    if (DebugConstants.enableArmMessages) {
      System.out.println(message);
    }
  }

  public static Command message(String message) {
    //    return debug.message(message);
    if (DebugConstants.enableArmMessages) {
      return Commands.print(message);
    } else {
      return Commands.none();
    }
  }

  public static Command start() {
    //    return debug.start();
    if (DebugConstants.enableArmMessages) {
      return Commands.print(debug.getClassString() + ":Start");
    } else {
      return Commands.none();
    }
  }

  public static Command end() {
    //    return debug.end();
    if (DebugConstants.enableArmMessages) {
      return Commands.print(debug.getClassString() + ":End");
    } else {
      return Commands.none();
    }
  }
}
