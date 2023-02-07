package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivetrain;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.SmoothingFilter;

public class WaypointCommand extends Command {

    private final double ACCEPTANCE_RADIUS = 0.1;
    private Drivetrain drive;
    private List<Pose2d> poses;
    private SmoothingFilter filter;
    private AutoDrive autoDrive;

    public WaypointCommand(Drivetrain drive) {
        poses = new ArrayList<>();
        this.drive = drive;
        autoDrive = new AutoDrive(2.0, 0, 0, 
                                2.0, 0, 0, 
                                0.5, 0.3, 0, 
                                1, 0.9, ACCEPTANCE_RADIUS);
    }

    public WaypointCommand addWaypoint(Pose2d pose) {
        poses.add(pose);
        return this;
    }

    public WaypointCommand addSmoothingFilter(SmoothingFilter sf) {
        filter = sf;
        return this;
    }

    @Override
    public void initialize() {
        autoDrive.initWaypoints();
        for (Pose2d pose : poses) {
            autoDrive.addWaypoint(pose);
        }
    }

    @Override
    public boolean execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(autoDrive.moveToWayPoint(drive.getCurrentPos()), drive.getGyroscopeRotation());
        drive.drive(filter == null ? speeds : filter.smooth(speeds));

        SmartDashboard.putNumber("Auto/Speed X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Auto/Speed Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Auto/Speed Omega", speeds.omegaRadiansPerSecond);

        return autoDrive.finishedAllWaypoints() && autoDrive.getDistance(drive.getCurrentPos(), poses.get(poses.size() - 1)) <= ACCEPTANCE_RADIUS;
    }

    @Override
    public void shutdown() {
        SmartDashboard.putNumber("Auto/Speed X", 0d);
        SmartDashboard.putNumber("Auto/Speed Y", 0d);
        SmartDashboard.putNumber("Auto/Speed Omega", 0d);
        drive.drive(new ChassisSpeeds());
    }

    
    
}
