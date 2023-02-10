package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivetrain;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.SmoothingFilter;
import frc.robot.autonomous.Waypoint;

public class WaypointCommand extends Command {
    // private final double ACCEPTANCE_RADIUS = 0.1;
    // private List<Pose2d> poses;
    // private SmoothingFilter filter;
    // private AutoDrive autoDrive;

    private Drivetrain drive;

    public enum SmoothingType {
        OFF(0),
        LOW(5),
        MEDIUM(10),
        HEAVY(15);

        private int size;
        SmoothingType(int size) {
            this.size = size;
        }

        public int getSmoothingSize() {
            return size;
        }
    }

    private Pose2d startPose;
    private Queue<Waypoint> waypoints;

    private HolonomicDriveController drivePID;
    private PIDController xPID;
    private PIDController yPID;
    private ProfiledPIDController turnPID;

    private final double ACCEPTABLE_TRANSLATION_ERROR_METERS = 0.1;
    private final double ACCEPTABLE_ROTATION_ERROR_RADIANS = Math.PI/90d;

    private SmoothingFilter smoother;

    // public WaypointCommand(Drivetrain drive) {
    //     poses = new ArrayList<>();
    //     this.drive = drive;
    //     autoDrive = new AutoDrive(2.0, 0, 0, 
    //                             2.0, 0, 0, 
    //                             0.5, 0.3, 0, 
    //                             1, 0.9, ACCEPTANCE_RADIUS);
    // }

    public WaypointCommand(Drivetrain drive, Pose2d startPose, SmoothingType type, Waypoint ... waypoints) {
        this.startPose = startPose;
        xPID = new PIDController(0.1, 0, 0);
        yPID = new PIDController(0.1, 0, 0);
        turnPID = new ProfiledPIDController(0.1, 0, 0, new Constraints(Math.PI, Math.PI));
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        drivePID = new HolonomicDriveController(xPID, yPID, turnPID);
        drivePID.setTolerance(
            new Pose2d(
            ACCEPTABLE_TRANSLATION_ERROR_METERS, 
            ACCEPTABLE_TRANSLATION_ERROR_METERS,
            new Rotation2d(ACCEPTABLE_ROTATION_ERROR_RADIANS))
        );

        int size = type.getSmoothingSize();
        smoother = new SmoothingFilter(size, size, size);
    }

    // public WaypointCommand addWaypoint(Pose2d pose) {
    //     poses.add(pose);
    //     return this;
    // }

    // public WaypointCommand addSmoothingFilter(SmoothingFilter sf) {
    //     filter = sf;
    //     return this;
    // }

    // @Override
    // public void initialize() {
    //     autoDrive.initWaypoints();
    //     for (Pose2d pose : poses) {
    //         autoDrive.addWaypoint(pose);
    //     }
    // }

    @Override
    public void initialize() {
        drive.resetPose(startPose);
    }

    @Override
    public boolean execute() {
        if (waypoints.size() != 0) {
            Waypoint currentWaypoint = waypoints.peek();
            Pose2d currentPose = drive.getCurrentPos();

            if (getLinearDistance(currentPose, currentWaypoint.getEndingPose()) <= ACCEPTABLE_TRANSLATION_ERROR_METERS) {
                waypoints.poll();
            }

            ChassisSpeeds speeds = drivePID.calculate(drive.getCurrentPos(), currentWaypoint.getEndingPose(), currentWaypoint.getEndingVelocity(), currentWaypoint.getEndingPose().getRotation());
            drive.drive(smoother.smooth(speeds));
        }
            
        return drivePID.atReference() && waypoints.size() == 0;
    }

    // @Override
    // public boolean execute() {
    //     // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(autoDrive.moveToWayPoint(drive.getCurrentPos()), drive.getGyroscopeRotation());
    //     // drive.drive(filter == null ? speeds : filter.smooth(speeds));

    //     // SmartDashboard.putNumber("Auto/Speed X", speeds.vxMetersPerSecond);
    //     // SmartDashboard.putNumber("Auto/Speed Y", speeds.vyMetersPerSecond);
    //     // SmartDashboard.putNumber("Auto/Speed Omega", speeds.omegaRadiansPerSecond);

    //     // return autoDrive.finishedAllWaypoints() && autoDrive.getDistance(drive.getCurrentPos(), poses.get(poses.size() - 1)) <= ACCEPTANCE_RADIUS;
    // }

    @Override
    public void shutdown() {
        SmartDashboard.putNumber("Auto/Speed X", 0d);
        SmartDashboard.putNumber("Auto/Speed Y", 0d);
        SmartDashboard.putNumber("Auto/Speed Omega", 0d);
        drive.drive(new ChassisSpeeds());
    }

    // Distance formula: (sqrt((x2 - x1)^2 + (y2 - y1)^2))
    private double getLinearDistance(Pose2d poseA, Pose2d poseB) {
        return Math.sqrt(Math.pow(poseA.getX() - poseB.getX(), 2) + Math.pow(poseA.getY() - poseB.getY(), 2));
    }
    
}
