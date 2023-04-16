package bhs.devilbotz.subsystems.drive;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Robot;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

public class DriveIOSim implements DriveIO {
  /**
   * The simulation model of our drivetrain
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html">Creating
   *     a Drivetrain Model</a>
   * @see <a
   *     href="https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java#L6">CTRE
   *     DifferentialDrive Simulation Example</a>
   * @since 1/31/2023
   */
  private final DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          // Create a linear system from our identification gains.
          Constants.SysIdConstants.PLANT,
          Constants.DriveConstants.MOTOR_CONFIGURATION,
          Robot.getDriveConstant("MOTOR_GEAR_RATIO").asDouble(),
          Robot.getDriveConstant("TRACK_WIDTH").asDouble(),
          Robot.getDriveConstant("WHEEL_RADIUS").asDouble(),

          // The standard deviations for measurement noise:
          // x and y:          0.001 m
          // heading:          0.001 rad
          // l and r velocity: 0.1   m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    sim.update(0.02);
    inputs.leftDistanceMeters = sim.getLeftPositionMeters();
    inputs.leftVelocityMetersPerSec = sim.getLeftVelocityMetersPerSecond();
    inputs.rightDistanceMeters = sim.getRightPositionMeters();
    inputs.rightVelocityMetersPerSec = sim.getRightVelocityMetersPerSecond();
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-sim.getHeading().getDegrees());

    inputs.yawPositionDeg = sim.getHeading().getDegrees();
    inputs.yawPositionRad = sim.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    sim.setInputs(leftVolts, rightVolts);
  }
}
