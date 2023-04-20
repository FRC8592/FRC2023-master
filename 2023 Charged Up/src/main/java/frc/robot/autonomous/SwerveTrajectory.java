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
    private Pose2d poseRobot = new Pose2d();

    private PIDController turnPID;

    private double maxRotationVelocity = Math.PI;
    private double turnDelay = 0.0;
    private boolean vision = false;
    private double acceptanceRange = 0.1;

    public SwerveTrajectory(Trajectory trajectory) {
        mXPID = new PIDController(1.0, 0, 0.0); // 0.1 0 -0.0002
        mYPID = new PIDController(1.0, 0, 0.0); // 0.1 0 -0.0002
        mTurnPID = new ProfiledPIDController(0.5, 0, 0, new Constraints(4 * Math.PI, 2 * Math.PI)); // Probably should increase the P value or maybe even change constraints to degrees
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);

        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);
        mTurnPID.enableContinuousInput(-Math.PI, Math.PI); // Might need to change to degrees

        turnPID = new PIDController(0.05, 0, 0);
        turnPID.setTolerance(0.1);
        // turnPID.enableContinuousInput(-Math.PI/2, Math.PI/2);

        mDrivePID.setTolerance(new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5)));

        // rotation = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
        rotation = new Rotation2d();
        mTrajectory = trajectory;
    }

    public SwerveTrajectory(Trajectory trajectory, boolean vision) {
        mXPID = new PIDController(1.0, 0, 0.0); // 0.1 0 -0.0002
        mYPID = new PIDController(1.0, 0, 0.0); // 0.1 0 -0.0002
        mTurnPID = new ProfiledPIDController(0.5, 0, 0, new Constraints(4 * Math.PI, 2 * Math.PI)); // Probably should increase the P value or maybe even change constraints to degrees
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);

        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);
        mTurnPID.enableContinuousInput(-Math.PI, Math.PI); // Might need to change to degrees

        turnPID = new PIDController(0.05, 0, 0);
        turnPID.setTolerance(0.1);
        // turnPID.enableContinuousInput(-Math.PI/2, Math.PI/2);

        mDrivePID.setTolerance(new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5)));

        // rotation = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
        rotation = new Rotation2d();
        mTrajectory = trajectory;

        this.vision = vision;
    }

    /**
     * @param rotation ending rotation
     * @return the same {@code SwerveTrajectory} object back but with the added {@code Rotation2d} for easy usage
     */
    public SwerveTrajectory addRotation(Rotation2d rotation) {
        this.rotation = Rotation2d.fromDegrees(rotation.getDegrees());
        return this;
    }

    /**
     * @param rotation ending rotation
     * @return the same {@code SwerveTrajectory} object back but with the added {@code Rotation2d} for easy usage
     */
    public SwerveTrajectory addRotation(Rotation2d rotation, double turnSpeed, double delay) {
        this.rotation = Rotation2d.fromDegrees(rotation.getDegrees());
        turnDelay = delay;
        maxRotationVelocity = turnSpeed;
        return this;
    }

    /**
     * @param rotation ending rotation
     * @return the same {@code SwerveTrajectory} object back but with the added {@code Rotation2d} for easy usage
     */
    public SwerveTrajectory addRotation(Rotation2d rotation, double delay) {
        this.rotation = Rotation2d.fromDegrees(rotation.getDegrees());
        turnDelay = delay;
        return this;
    }

    public SwerveTrajectory setMaxRotationSpeed(double radiansPerSecond) {
        maxRotationVelocity = radiansPerSecond;
        return this;
    }

    public SwerveTrajectory setTrajectoryConfiguration(TrajectoryConfig config) {
        this.config = config;
        return this;
    }

    public double getConfiguredEndingVelocity() {
        return config.getEndVelocity();
    }

    public SwerveTrajectory addVision() {
        this.vision = true;
        return this;
    }

    public SwerveTrajectory setAcceptanceRange(double acceptance) {
        acceptanceRange = acceptance;
        return this;
    }

    /**
     * @param pSeconds current time in current {@code SwerveTrajectory} (sec)
     * @param robotPose current {@code Pose2d} of robot
     * @return a {@code ChassisSpeeds} object for x, y, and omega speeds in imperial units (m/s and rad/s)
     */
    public ChassisSpeeds sample(double pSeconds, Pose2d robotPose) {
        State state = mTrajectory.sample(pSeconds);

        // if (DriverStation.getAlliance() == Alliance.Red) {
        //     this.rotation = Rotation2d.fromDegrees(180 - rotation.getDegrees());
        // }

        ChassisSpeeds desired = mDrivePID.calculate(getInitialPose(), state, rotation);
        if (Robot.isReal()) {
            desired = mDrivePID.calculate(
                robotPose, 
                state, 
                rotation
            );

            // desired = ChassisSpeeds.fromFieldRelativeSpeeds(desired, robotPose.getRotation());
            // SmartDashboard.putNumber("Current Heading", robotPose.getRotation().getDegrees());
        }

        // SmartDashboard.putNumber("Error X", getEndingPose().getX() - robotPose.getX());
        // SmartDashboard.putNumber("Error Y", getEndingPose().getY() - robotPose.getY());
        // SmartDashboard.putNumber("Error Theta", rotation.getDegrees() - robotPose.getRotation().getDegrees());

        // SmartDashboard.putNumber("Ending X", getEndingPose().getX());
        // SmartDashboard.putNumber("Ending Y", getEndingPose().getY());
        // SmartDashboard.putNumber("Ending Theta", rotation.getDegrees());

        // SmartDashboard.putBoolean("AT SETPOINT", mDrivePID.atReference());

        poseRobot = robotPose;

        double turn = turnPID.calculate(0, getErrorAngle(robotPose, new Pose2d(0, 0, rotation)));
        turn = Math.max(-maxRotationVelocity, Math.min(maxRotationVelocity, pSeconds >= turnDelay ? turn : 0.0));

        desired = new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, -turn);
        return desired;
    }

    /**
     * @param time in seconds
     * @return whether the path has finished
     */
    public boolean isFinished(double time) {
        if (Robot.isReal()) {
            return 
                // ((Math.abs(getEndingPose().getX() - poseRobot.getX()) <= 0.1) &&
                // (Math.abs(getEndingPose().getY() - poseRobot.getY()) <= 0.1)) 
                // ||
                time >= mTrajectory.getTotalTimeSeconds() || (Math.abs(getEndingPose().getX() - poseRobot.getX()) <= acceptanceRange && vision);
        } else {
            return time >= mTrajectory.getTotalTimeSeconds();
        }
    }

    private double getErrorAngle(Pose2d robot, Pose2d goal){
        /*** Computation for currect rotate errors into waypoint ****/
       double goalAngle = goal.getRotation().getDegrees();
       double curAngle = robot.getRotation().getDegrees();
       if (curAngle < 0) {
            curAngle += 360;
       }
       double errorAngle = 0; 
       //we only use angles between 0 and 2 PI so convert the angles to that range.
       if(goalAngle < 0){
                   goalAngle += 360;    
       }
       // find shortest angle difference error angle should allways be > -PI and <= PI
       errorAngle = goalAngle - curAngle;   
       if(errorAngle > 180){
           errorAngle -= 360;
       } else if(errorAngle <= -180){
           errorAngle += 360;
       } 
       return errorAngle;
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

    /**
     * @return ending {@code Pose2d}
     */
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