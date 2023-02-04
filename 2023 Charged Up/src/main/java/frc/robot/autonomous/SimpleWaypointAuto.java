package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.WaypointCommand;

public class SimpleWaypointAuto extends BaseAuto {
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new WaypointCommand(drive)
                .addWaypoint(new Pose2d(0,4, Rotation2d.fromDegrees(-180)))
                // .addWaypoint(new Pose2d(-1,-1, Rotation2d.fromDegrees(180)))
                // .addWaypoint(new Pose2d(-1,0, Rotation2d.fromDegrees(270)))
                // .addWaypoint(new Pose2d(0,0, Rotation2d.fromDegrees(360)))
                // .addSmoothingFilter(new SmoothingFilter(5, 5, 5))
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
        SmartDashboard.putString("Robot Pose", drive.getCurrentPos().toString());
        
    }

}
