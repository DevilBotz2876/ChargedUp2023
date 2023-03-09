package bhs.devilbotz.commands;

import bhs.devilbotz.utils.Debug;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandDebug {
  static final Debug debug = new Debug("Command");

  public static void trace() {
    //    debug.trace();
    System.out.println(debug.getTraceString());
  }

  public static void trace(String message) {
    System.out.println(debug.getTraceString() + ":" + message);
  }

  public static void println(String message) {
    System.out.println(message);
  }

  public static Command message(String message) {
    //    return debug.message(message);
    return Commands.print(message);
  }

  public static Command start() {
    //    return debug.start();
    return Commands.print(debug.getClassString() + ":Start");
  }

  public static Command end() {
    //    return debug.end();
    return Commands.print(debug.getClassString() + ":End");
  }
}
