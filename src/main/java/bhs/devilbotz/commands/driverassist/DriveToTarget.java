package bhs.devilbotz.commands.driverassist;

import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.utils.PhotonCameraWrapper;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToTarget extends CommandBase {
  private final DriveTrain drive;

  public DriveToTarget(DriveTrain drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d estimatedPose = drive.getPose();
    PhotonCameraWrapper pcw = drive.getPhotonCameraWrapper();
    Pose2d targetPose = pcw.getClosestTarget(estimatedPose);

    // move the target back 2 feet
    Translation2d twoFeetBack =
        targetPose.getTranslation().plus(new Translation2d(Units.feetToMeters(3.5), 0));

    Rotation2d flippedRotation = targetPose.getRotation().plus(Rotation2d.fromDegrees(180));

    PathPlannerTrajectory traj1 =
        PathPlanner.generatePath(
            new PathConstraints(0.25, 0.3),
            new PathPoint(
                estimatedPose.getTranslation(), estimatedPose.getRotation()), // position, heading
            new PathPoint(twoFeetBack, flippedRotation) // position, heading
            );
    System.out.println("path generated");
    drive.addPathToField(traj1);
    // execute
    drive.followTrajectoryCommand(traj1, true).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
