package bhs.devilbotz.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraManager extends SubsystemBase {
    UsbCamera gripperCamera;
    /**
     * Creates a new CameraManager.
     */
    public CameraManager() {
        gripperCamera = CameraServer.startAutomaticCapture(0);
        gripperCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320/2, 240/2, 10);

        // Add gripper camera to shuffleboard
    }

    /**
     * This method updates once per loop of the robot.
     *
     * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
     *     Based Programming</a>
     */
    @Override
    public void periodic() {

    }
}
