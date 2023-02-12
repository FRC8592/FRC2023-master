package frc.robot.autonomous.blue;

import frc.robot.autonomous.BaseAuto;
import frc.robot.autonomous.Waypoint;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.WaypointCommand;
import frc.robot.commands.WaypointCommand.SmoothingType;

import static frc.robot.autonomous.AutonomousPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueCableCoverWaypointAuto extends BaseAuto {
    @Override
    public void initialize() {
        queue = new CommandQueue(
            // new WaypointCommand(
            //     drive, 
            //     GRID_H.getPose(),
            //     SmoothingType.LOW, 
            //     new Waypoint(firstPose.getX(), firstPose.getY(), Rotation2d.fromDegrees(0), 0)
            // )
            new WaypointCommand(drive, SmoothingType.OFF)
                // .addWaypoint(INTERMEDIARY_CABLE_COVER.getPose())
                .addWaypoint(new Pose2d(2,0, Rotation2d.fromDegrees(90)))
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
