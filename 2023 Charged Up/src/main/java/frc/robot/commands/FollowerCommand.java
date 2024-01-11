package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.Vision;
import frc.robot.autonomous.SwerveTrajectory;

public class FollowerCommand extends Command {
    private Drivetrain drive;
    private SwerveTrajectory trajectory;
    private Timer timer;
    private Vision vision;
    private PIDController visionPID;
    // private Rotation2d endRotation;

    public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj) {
        drive = pDrive;
        trajectory = pTraj;
        timer = new Timer();
    }

    public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj, String tag) {
        drive = pDrive;
        trajectory = pTraj;
        timer = new Timer();
        setTag(tag);
    }

    public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj, Rotation2d pRot) {
        drive = pDrive;
        trajectory = pTraj.addRotation(pRot);
        timer = new Timer();

        // endRotation = pRot;
    }

    public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj, Rotation2d pRot, String tag) {
        drive = pDrive;
        trajectory = pTraj.addRotation(pRot);
        timer = new Timer();
        setTag(tag);

        // endRotation = pRot;
    }

    public FollowerCommand(Drivetrain drive, Vision vision, SwerveTrajectory pTraj) {
        this.drive = drive;
        this.vision = vision;
        trajectory = pTraj;
        visionPID = new PIDController(0.05, 0.0, 0.0);
        timer = new Timer();
    }

    // public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj, Rotation2d pRot, boolean lockWheels) {
    //     drive = pDrive;
    //     trajectory = pTraj.addRotation(pRot);
    //     timer = new Timer();
    //     setTag(tag);

    //     endRotation = pRot;
    // }

    // public FollowerCommand(Drivetrain pDrive, SwerveTrajectory pTraj, Rotation2d pRot, boolean lockWheels, String tag) {
    //     drive = pDrive;
    //     trajectory = pTraj.addRotation(pRot);
    //     timer = new Timer();
    //     setTag(tag);

    //     endRotation = pRot;
    // }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        ChassisSpeeds speeds = trajectory.sample(timer.get(), drive.getCurrentPos());
        double time = timer.get();

        if (!Robot.isReal()) {
            simulateRobotPose(trajectory.trajectory().sample(time).poseMeters, speeds);
        }
        
        ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        if (vision != null) {
            double vyVision = vision.lockTargetSpeed( 0,   visionPID,"tx",   1.0, 0.0);
            if (Math.abs(trajectory.trajectory().sample(trajectory.trajectory().getTotalTimeSeconds()).poseMeters.getX() - drive.getCurrentPos().getX()) <= 0.2) {
                vyVision = 0;
            }

            // SmartDashboard.putNumber("VY Speed From Vision", vyVision);

            newSpeeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
               -vyVision , 
                speeds.omegaRadiansPerSecond
            );
        }
        drive.drive(newSpeeds);
        
        return trajectory.isFinished(time);
    }

    @Override
    public Pose2d getStartPose() {
        return trajectory.getInitialPose();
    }

    private void simulateRobotPose(Pose2d pose, ChassisSpeeds desiredSpeeds) {
        Trajectory traj = trajectory.trajectory();
        Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), traj.sample(traj.getTotalTimeSeconds()).poseMeters.getRotation()));

        // SmartDashboard.putNumber("Field Relative X Velocity", desiredSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("Field Relative Y Velocity", desiredSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("Field Relative Omega", desiredSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void shutdown() {
        // if (lockWheels) {
        //     drive.setWheelLock();
        // } else {
            drive.drive(new ChassisSpeeds());
        // }
        // drive.drive(trajectory.sample(trajectory.trajectory().getTotalTimeSeconds() - 0.02, drive.getCurrentPos()));
    }
}