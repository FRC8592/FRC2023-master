package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.WaypointCommand;
import frc.robot.commands.WaypointCommand.SmoothingType;

public class IntermediateWaypointAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new WaypointCommand(drive, new Pose2d(), SmoothingType.OFF,
                new Waypoint(0, 1, Rotation2d.fromDegrees(0), 0),
                new Waypoint(2, 0, Rotation2d.fromDegrees(0), 0.25),
                new Waypoint(1, 1, Rotation2d.fromDegrees(0), 0.75),
                new Waypoint(0, 0, Rotation2d.fromDegrees(0), 0)
            )
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
