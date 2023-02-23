package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

public class MoveOutCommunityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory MOVE_OUT = generateTrajectoryFromPoints(
        config,
        GRID_A.getPose(),
        GRID_A.translate(2, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, MOVE_OUT)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}