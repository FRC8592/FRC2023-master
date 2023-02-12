package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Odometry {
    SwerveDrivePoseEstimator poseEstimator;
    AprilTags aprilTags;
    Vision vision;
    Drivetrain drive;

    public void Odometry(Vision vision, AprilTags aprilTags,SwerveDrivePoseEstimator poseEstimator, Drivetrain drivetrain){
        this.aprilTags = aprilTags;
        this.poseEstimator = poseEstimator;
        this.drive = drivetrain;
        this.vision = vision;

        poseEstimator = new SwerveDrivePoseEstimator(drivetrain.m_kinematics, drivetrain.getGyroscopeRotation(), 
        new SwerveModulePosition[]{
            drivetrain.getSMPosition(drivetrain.m_frontLeftModule), 
            drivetrain.getSMPosition(drivetrain.m_frontRightModule),
            drivetrain.getSMPosition(drivetrain.m_backLeftModule),
            drivetrain.getSMPosition(drivetrain.m_backRightModule)
        }, drivetrain.getCurrentPos());


      

    }

    public void update(){

    }
}
