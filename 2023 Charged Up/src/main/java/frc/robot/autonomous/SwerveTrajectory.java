package frc.robot.autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Custom Trajectory generator class based on the WPILib {@code Trajectory} class
 */
public class SwerveTrajectory {
    private HolonomicDriveController mDrivePID;
    private PIDController mXPID, mYPID;
    private ProfiledPIDController mTurnPID;
    private Rotation2d rotation;
    private Trajectory mTrajectory;
    private TrajectoryConfig config = new TrajectoryConfig(0, 0).setEndVelocity(0).setStartVelocity(0);

    public SwerveTrajectory(Trajectory trajectory) {
        mXPID = new PIDController(0.1, 0, 0.0); // 0.1 0 -0.0002
        mYPID = new PIDController(0.1, 0, 0.0); // 0.1 0 -0.0002
        mTurnPID = new ProfiledPIDController(0.2, 0, 0.0, new Constraints(4 * Math.PI, 2 * Math.PI)); // Probably should increase the P value or maybe even change constraints to degrees
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);

        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.001, 0.1);
        mTurnPID.enableContinuousInput(-Math.PI, Math.PI); // Might need to change to degrees

        mDrivePID.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5)));

        // rotation = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
        rotation = new Rotation2d();
        mTrajectory = trajectory;
    }

    /**
     * @param rotation ending rotation
     * @return the same {@code SwerveTrajectory} object back but with the added {@code Rotation2d} for easy usage
     */
    public SwerveTrajectory addRotation(Rotation2d rotation) {
        this.rotation = Rotation2d.fromDegrees(-rotation.getDegrees());
        return this;
    }

    public SwerveTrajectory setTrajectoryConfiguration(TrajectoryConfig config) {
        this.config = config;
        return this;
    }

    public double getConfiguredEndingVelocity() {
        return config.getEndVelocity();
    }

    /**
     * @param pSeconds current time in current {@code SwerveTrajectory} (sec)
     * @param robotPose current {@code Pose2d} of robot
     * @return a {@code ChassisSpeeds} object for x, y, and omega speeds in imperial units (m/s and rad/s)
     */
    public ChassisSpeeds sample(double pSeconds, Pose2d robotPose) {
        State state = mTrajectory.sample(pSeconds);

        ChassisSpeeds desired = mDrivePID.calculate(getInitialPose(), state, rotation);
        if (Robot.isReal()) {
            desired = mDrivePID.calculate(
                robotPose, 
                state, 
                rotation
            );

            // desired = ChassisSpeeds.fromFieldRelativeSpeeds(desired, robotPose.getRotation());
            SmartDashboard.putNumber("Current Heading", robotPose.getRotation().getDegrees());
        }

        SmartDashboard.putNumber("Error X", getEndingPose().getX() - robotPose.getX());
        SmartDashboard.putNumber("Error Y", getEndingPose().getY() - robotPose.getY());
        SmartDashboard.putNumber("Error Theta", getEndingPose().getRotation().getDegrees() - robotPose.getRotation().getDegrees());

        SmartDashboard.putBoolean("AT SETPOINT", mDrivePID.atReference());

        // SmartDashboard.putNumber("Desired X", desired.vxMetersPerSecond);
        // SmartDashboard.putNumber("Desired Y", desired.vyMetersPerSecond);
        // SmartDashboard.putNumber("Desired Omega", desired.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("Ending Turn", rotation.getDegrees());
        // SmartDashboard.putNumber("Starting Rotation", trajectory().getInitialPose().getRotation().getDegrees());
        return desired;
    }

    public boolean isFinished(double time) {
        if (Robot.isReal()) {
            // return mDrivePID.atReference() && time >= (mTrajectory.getTotalTimeSeconds());
            return time >= (mTrajectory.getTotalTimeSeconds());
        } else {
            // return mDrivePID.atReference();
            return time >= mTrajectory.getTotalTimeSeconds();
        }
    }

    /**
     * @return underlying WPILib {@code Trajectory} object that the current {@code SwerveTrajectory} is based on
     */
    public Trajectory trajectory() {
        return mTrajectory;
    }

    /**
     * @return ending {@code Rotation2d}
     */
    public Rotation2d getEndingRotation() {
        return rotation;
    }

    public Pose2d getEndingPose() {
        return mTrajectory.sample(mTrajectory.getTotalTimeSeconds() - 0.02).poseMeters;
    }

    /**
     * @return initial {@code Pose2d}
     */
    public Pose2d getInitialPose() {
        return mTrajectory.getInitialPose();
    }
}