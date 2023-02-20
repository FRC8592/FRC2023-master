
package frc.robot.autonomous.autons;

import frc.robot.commands.CommandQueue;
import frc.robot.commands.WaypointCommand;
import frc.robot.commands.WaypointCommand.SmoothingType;

import static frc.robot.autonomous.AutonomousPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class WaypointBox extends BaseAuto {
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new WaypointCommand(drive, SmoothingType.OFF)
                .setStartingWaypoint(new Pose2d())
                .addWaypoint(new Pose2d(new Translation2d(-2, 0), new Rotation2d(0)))
                .addWaypoint(new Pose2d(new Translation2d(-2,2), new Rotation2d(180)))
                .addWaypoint(new Pose2d(new Translation2d(0,2), new Rotation2d(180)))
                .addWaypoint(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}