package frc.robot.autonomous.trajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SwerveTrajectory {
    private HolonomicDriveController mDrivePID;
    private PIDController mXPID, mYPID;
    private ProfiledPIDController mTurnPID;
    private Rotation2d mRotation;
    private boolean mRelative;

    private Trajectory mTrajectory;

    public SwerveTrajectory(Trajectory trajectory, Rotation2d pRotation, boolean pRelative) {
        mXPID = new PIDController(0.01, 0, 0);
        mYPID = new PIDController(0.01, 0, 0);
        mTurnPID = new ProfiledPIDController(1, 0, 0, new Constraints(Math.PI, Math.PI/2));
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);

        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);
        
        mRotation = pRotation;
        mRelative = pRelative;
        mTrajectory = trajectory;
    }

    public SwerveTrajectory(Trajectory trajectory) {
        mXPID = new PIDController(0.01, 0, 0);
        mYPID = new PIDController(0.01, 0, 0);
        mTurnPID = new ProfiledPIDController(0.01, 0, 0, new Constraints(Math.PI/2, Math.PI/2));
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);

        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);

        mRotation = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
        mRelative = false;
        mTrajectory = trajectory;
    }

    public ChassisSpeeds sample(double pSeconds, Pose2d robotPose) {
        State state = mTrajectory.sample(pSeconds);


        if (Robot.isReal()) {
            SmartDashboard.putNumber("Desired Turn", mDrivePID.calculate(
                robotPose, 
                state, 
                mRelative ? mRotation.plus(getInitialPose().getRotation()) : mRotation
            ).omegaRadiansPerSecond);
            return mDrivePID.calculate(
                robotPose, 
                state, 
                mRelative ? mRotation.plus(getInitialPose().getRotation()) : mRotation
            );
        } else {
            return mDrivePID.calculate(new Pose2d(), state, new Rotation2d());
        }
    }

    public Trajectory trajectory() {
        return mTrajectory;
    }

    public Rotation2d getEndingRotation() {
        return mRotation;
    }

    public Pose2d getInitialPose() {
        return mTrajectory.getInitialPose();
    }
}
