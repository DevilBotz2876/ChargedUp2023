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
            case SET_RED:
                arduino.sendMessage( new byte[] {0x1});
                break;
            case SET_BLUE:
                arduino.sendMessage( new byte[] {0x2});
                break;
            case CLEAR:
                arduino.sendMessage( new byte[] {0x0});
                break;
        }



        sentMessage = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return sentMessage;
    }
}
