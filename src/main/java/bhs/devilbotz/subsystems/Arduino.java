package bhs.devilbotz.subsystems;

import bhs.devilbotz.utils.Alert;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arduino extends SubsystemBase {
  private SerialPort arduino;
  private boolean exists = false;

  public Arduino() {

    // Connect ot one of three different USB ports
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
      System.out.println("---------Connected on kUSB---------");
      exists = true;
    } catch (Exception e) {

      try {
        arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
        System.out.println("---------Connected on kUSB1---------");
        exists = true;
      } catch (Exception e1) {

        try {
          arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
          System.out.println("---------Connected on kUSB2---------");
          exists = true;
        } catch (Exception e2) {
          exists = false;
          System.out.println("Failed to connect to USB Ports");
          new Alert("Failed to connect to camera! Check USB Connection", Alert.AlertType.WARNING)
              .set(true);
        }
      }
    }
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public void sendMessage(byte[] info) {
    if (exists) {
      arduino.write(info, 1);
    }
  }
}
