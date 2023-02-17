// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.utils;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.file.Files;
import java.nio.file.Paths;

/** Add your docs here. */
public class RobotConfig {

  private static JsonNode robotConfig;

  private static final String robotUniqueIdDefault =
      "00802F17DEE0"; // This is the competition bot's unique ID (MAC address)

  private static final String compId = "00802F17DEE0";
  private static final String practiceId = "00802F17C8A9";

  private static String curId = null;

  public static boolean isCompBot() {
    if (curId == null) {
      loadConfig();
    }
    return (compId.equalsIgnoreCase(curId));
  }

  public static void loadConfig() {
    try {
      String robotConfigPath = getRobotConfigFilePath();
      File robotConfigFile = new File(robotConfigPath);
      ObjectMapper objectMapper = new ObjectMapper();
      robotConfig = objectMapper.readTree(robotConfigFile);
      System.out.println("Loaded robot config: " + robotConfigPath);
      System.out.println("\tname: " + robotConfig.get("name").asText());
      System.out.println("\tid: " + robotConfig.get("id").asText());
      curId = robotConfig.get("id").asText();
    } catch (Exception ex) {
      System.out.println(ex.toString());
    }
  }

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
    String robotConfigFilePathSuffix = ".json";

    if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix))) {
      return robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix;
    } else if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix))) {
      System.out.println(
          "WARNING: robotConfig for "
              + robotUniqueId
              + " not found, using default robot "
              + robotUniqueIdDefault);
      return robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix;
    } else {
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

      StringBuilder macString = new StringBuilder();
      for (byte m : mac) {
        macString.append(String.format("%02X", m).replace("-", ""));
      }
      return macString.toString();
      // TODO: Implement checking for the practice bot
    } catch (SocketException | UnknownHostException e) {
      throw new RuntimeException(e);
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
}
