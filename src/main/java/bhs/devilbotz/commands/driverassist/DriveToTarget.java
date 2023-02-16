package bhs.devilbotz.commands.driverassist;

import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.PhotonCameraWrapper;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class DriveToTarget extends CommandBase {
    private final DriveTrain drive;

    public DriveToTarget(DriveTrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d estimatedPose = drive.getEstimatedPose();
        PhotonCameraWrapper pcw = drive.getPhotonCameraWrapper();
        Pose2d targetPose = pcw.getClosestTarget(estimatedPose);

        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(estimatedPose.getTranslation(), estimatedPose.getRotation()), // position, heading
                new PathPoint(targetPose.getTranslation(), targetPose.getRotation()) // position, heading
        );
        drive.addPathToField(traj1);
        // Generate a simple trajectory
        // Simple path without holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2

        // Create pose estimator


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
