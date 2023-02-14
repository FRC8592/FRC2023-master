package frc.robot.autonomous.autons;

import frc.robot.commands.CommandQueue;
import frc.robot.commands.WaypointCommand;
import frc.robot.commands.WaypointCommand.SmoothingType;

import static frc.robot.autonomous.AutonomousPositions.*;

public class WaypointTestingAuto extends BaseAuto {
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new WaypointCommand(drive, SmoothingType.OFF)
                .setStartingWaypoint(GRID_I.getPose())
                .addWaypoint(INTERMEDIARY_LOADING_ZONE.getPose())
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
