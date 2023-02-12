package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.autonomous.trajectory.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;

public class CoordinateBasedAuto extends BaseAuto {
    TrajectoryConfig config = new TrajectoryConfig(5, 3);

    private Trajectory A5_TO_ILZ = TrajectoryGenerator.generateTrajectory(
        AutonomousPositions.GRID_A.getPose(), 
        List.of(
            AutonomousPositions.INTERMEDIARY_CABLE_COVER.getTranslation()
        ), 
        AutonomousPositions.GAME_PIECE_1.getPose(),
        config
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, new SwerveTrajectory(A5_TO_ILZ))
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
