package bhs.devilbotz.commands.led;

import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.Arduino;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLEDMode extends CommandBase {
  Arduino arduino;
  boolean sentMessage;
  LEDModes ledMode;

  public SetLEDMode(Arduino arduino, LEDModes ledMode) {
    this.arduino = arduino;
    this.ledMode = ledMode;
  }

  @Override
  public void initialize() {
    switch (ledMode) {
      case CLEAR:
        arduino.sendMessage(new byte[] {0x0});
        break;
      case SET_RED:
        arduino.sendMessage(new byte[] {0x1});
        break;
      case SET_BLUE:
        arduino.sendMessage(new byte[] {0x2});
        break;
      case SET_CONE:
        arduino.sendMessage(new byte[] {0x3});
        break;
      case SET_CUBE:
        arduino.sendMessage(new byte[] {0x4});
        break;
      case SET_AUTONOMOUS:
        arduino.sendMessage(new byte[] {0x5});
        break;
      case SET_ARM_UP:
        arduino.sendMessage(new byte[] {0x6});
        break;
      case SET_ARM_DOWN:
        arduino.sendMessage(new byte[] {0x7});
        break;
      case SET_ARM_IDLE:
        arduino.sendMessage(new byte[] {0x8});
        break;
    }

    sentMessage = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return sentMessage;
  }

  // runs while disabled true
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
