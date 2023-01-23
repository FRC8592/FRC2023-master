package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryFollower {
    
    private SwerveTrajectory mTrajectory;
    private double prevTime;

    public TrajectoryFollower(SwerveTrajectory pTrajectory) {
        mTrajectory = pTrajectory;
        prevTime = 0;
    }

    public ChassisSpeeds follow(Pose2d robotPose, double pTime, boolean targetLock) {
        ChassisSpeeds desiredSpeeds = mTrajectory.sample(pTime, robotPose);

        if (!Robot.isReal()) {
            simulateRobotPose(mTrajectory.trajectory().sample(pTime).poseMeters, desiredSpeeds);
        }

        prevTime = pTime;
        return desiredSpeeds;
    }

    private void simulateRobotPose(Pose2d pose, ChassisSpeeds desiredSpeeds) {
        Trajectory traj = mTrajectory.trajectory();
        Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), traj.sample(traj.getTotalTimeSeconds()).poseMeters.getRotation()));

        SmartDashboard.putNumber("Field Relative X Velocity", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Y Velocity", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Omega", desiredSpeeds.omegaRadiansPerSecond);
    }

    public boolean finished() {
        return prevTime >= mTrajectory.trajectory().getTotalTimeSeconds();
    }

}