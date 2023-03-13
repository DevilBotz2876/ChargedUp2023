package bhs.devilbotz.utils;

import bhs.devilbotz.utils.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Debug {
  private final int level = 3;
  private final String group;

  public Debug() {
    this("Alerts");
  }

  public Debug(String group) {
    this.group = group;
  }

  private static String getCallerClass(int level) {
    return Thread.currentThread().getStackTrace()[level + 1].getClassName();
  }

  private static String getCallerMethod(int level) {
    return Thread.currentThread().getStackTrace()[level + 1].getMethodName();
  }

  private static String getCaller(int level) {
    return getCallerClass(level + 1) + ":" + getCallerMethod(level + 1);
  }

  public void trace() {
    new Alert(group, getCaller(this.level), AlertType.INFO).set(true);
  }

  public String getTraceString() {
    return getCaller(this.level);
  }

  public String getClassString() {
    return getCallerClass(this.level);
  }

  public Command message(String message) {
    return new InstantCommand(
        () -> {
          new Alert(group, message, AlertType.INFO).set(true);
        });
  }

  public Command start() {
    return message(getCallerClass(this.level) + ":Start");
  }

  public Command end() {
    return message(getCallerClass(this.level) + ":End");
  }
}
