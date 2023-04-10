// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.Alert;
import bhs.devilbotz.utils.ShuffleboardManager;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private AutonomousModes autoMode;

  private ShuffleboardManager shuffleboardManager;
  private RobotContainer robotContainer;
  private static JsonNode robotConfig;
  private DriverStation.Alliance currentAlliance = null;

  /**
   * We default to using the "Competition BOT" robot ID if the current ID is not found. This is
   * useful for simulation
   */
  private static final String robotUniqueIdDefault =
      "00802F17DEE0"; // This is the competition bot's unique ID (MAC address)

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    try {
      String robotConfigPath = getRobotConfigFilePath();
      File robotConfigFile = new File(robotConfigPath);
      ObjectMapper objectMapper = new ObjectMapper();
      robotConfig = objectMapper.readTree(robotConfigFile);
      System.out.println("Loaded robot config: " + robotConfigPath);
      new Alert(
              "Robot Name: "
                  + robotConfig.get("name").asText()
                  + " Id: "
                  + robotConfig.get("id").asText(),
              Alert.AlertType.INFO)
          .set(true);
    } catch (Exception ex) {
      System.out.println(ex.toString());
      new Alert("Failed to load robot config. Robot will not function", Alert.AlertType.ERROR)
          .set(true);
    }

    robotContainer = new RobotContainer();
    shuffleboardManager = robotContainer.getShuffleboardManager();

    if (Robot.checkCapability("hasCamera")) {
      UsbCamera armCamera = CameraServer.startAutomaticCapture(0);

      armCamera.setResolution(240, 135);
      armCamera.setFPS(20);
      armCamera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
    } else {
      Alert cameraAlert =
          new Alert("Robot is a simulation. Camera will display black.", Alert.AlertType.WARNING);
      cameraAlert.set(true);
      // Create a blank video source to simulate the camera
      CvSource armCameraSource = CameraServer.putVideo("Test", 320, 240);
      armCameraSource.putFrame(new Mat(320, 240, 0));

      ShuffleboardManager.putCamera(armCameraSource);
    }
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics that
   * you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    autoMode = ShuffleboardManager.autoModeChooser.getSelected();
    shuffleboardManager.updateValues();
  }

  /** This method is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.resetRobotPosition();
    // Wait for the voltage to appear
    if (RobotController.getBatteryVoltage() < Constants.MIN_BATTERY_VOLTAGE) {
      new Alert("Battery voltage is low", Alert.AlertType.WARNING).set(true);
      robotContainer.setLEDMode(LEDModes.SET_VOLTAGE_WARNING);
    } else {
      robotContainer.setLEDMode(LEDModes.SET_LOADING);
    }
  }

  @Override
  public void disabledPeriodic() {
    // We check if the alliance has changed in disable perioic update the LEDs accordingly.  We
    // remember what the LED was last set to to prevent constantly sending the LED commands.
    if (DriverStation.getAlliance() != currentAlliance) {
      currentAlliance = DriverStation.getAlliance();
      robotContainer.setLEDModeAlliance();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.setLEDMode(LEDModes.SET_AUTONOMOUS);
    robotContainer.resetRobotPosition();

    autonomousCommand = robotContainer.getAutonomousCommand(autoMode);
    Gripper.enableCompressor();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This method is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.resetRobotPosition();
    robotContainer.setLEDModeAlliance();
    robotContainer.resetRobotPosition();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Gripper.enableCompressor();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // Every time the robot is enabled in teleop initialize gripper to same/set position.
    // We want the gripper to be in same starting position every time
    // so driver does not have to remember/see what gripper position is.
    // If there is not enough stored air in tanks then this will do nothing.
    robotContainer.initGripper();
  }

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shuffleboardManager.updateValuesTeleop();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This method is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    Alert simulationAlert = new Alert("Robot is a simulation", Alert.AlertType.INFO);
    simulationAlert.set(true);
  }

  /** This method is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * This method returns the expected JSON config for the current robot. The current robot is
   * identified by getting the MAC address. If the robot specific config is not found, it loads the
   * default config
   *
   * @return The path of the robot specific JSON config file
   * @throws Exception If a valid JSON config file isn't found
   * @see {@link #robotUniqueIdDefault}
   */
  private static String getRobotConfigFilePath() throws Exception {
    String robotConfigFilePathPrefix =
        Filesystem.getDeployDirectory() + File.separator + "robotconfig" + File.separator;
    String robotUniqueId = getMacAddress();
    if (Robot.isSimulation()) {
      robotUniqueId = "simulation";
    }
    String robotConfigFilePathSuffix = ".json";

    if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix))) {
      return robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix;
    } else if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix))) {
      new Alert(
              "WARNING: robotConfig for "
                  + robotUniqueId
                  + " not found, using default robot "
                  + robotUniqueIdDefault,
              Alert.AlertType.WARNING)
          .set(true);
      return robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix;
    } else {
      new Alert(
              "Can't find a valid robot config JSON file in: " + robotConfigFilePathPrefix,
              Alert.AlertType.ERROR)
          .set(true);
      throw new Exception(
          "Can't find a valid robot config JSON file in: " + robotConfigFilePathPrefix);
    }
  }

  /**
   * This method returns the current robot's mac address. In simulation, it returns the current
   * computer's mac address.
   *
   * @return the current robot's mac address as a HEX string
   * @author ParkerMeyers
   * @see <a
   *     href=https://github.com/BHSRobotix/OffSeason2023/blob/main/src/main/java/bhs/devilbotz/subsystems/UniqueID.java#L26>@ParkerMeyers
   *     original code</a>
   */
  private static String getMacAddress() {
    try {
      NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());
      byte[] mac = network.getHardwareAddress();

      if (null != mac) {
        StringBuilder macString = new StringBuilder();
        for (byte m : mac) {
          macString.append(String.format("%02X", m).replace("-", ""));
        }
        return macString.toString();
      } else {
        throw new UnknownHostException("Cannot get mac address");
      }
    } catch (SocketException | UnknownHostException e) {
      new Alert("Can't get MAC address", Alert.AlertType.ERROR).set(true);
      /* Default to using competition bot */
      return robotUniqueIdDefault;
    }
  }

  /**
   * This method returns the requested robot specific system id configuration
   *
   * @param name The system id constant name
   * @return JsonNode containing the value of the requested configuration value
   */
  public static JsonNode getSysIdConstant(String name) {
    return robotConfig.get("sysid").get(name);
  }

  /**
   * This method returns the requested robot specific drive train configuration
   *
   * @param name The drivetrain constant name
   * @return JsonNode containing the value of the requested configuration value
   */
  public static JsonNode getDriveTrainConstant(String name) {
    return robotConfig.get("drivetrain").get(name);
  }

  /**
   * This method returns the requested robot specific capabilities
   *
   * @param name The capability name
   * @return true if the capability is supported, otherwise false
   */
  public static boolean checkCapability(String name) {
    try {
      return robotConfig.get("capabilities").get(name).asBoolean(false);
    } catch (Exception e) {
      return false;
    }
  }
}
