package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

public class MiddlePreloadParkAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    @Override
    public void initialize() {
        SwerveTrajectory E5_TO_BM = generateTrajectoryFromPoints(
            GRID_E, 
            BALANCE_MIDDLE, 
            Rotation2d.fromDegrees(0),
            config
        );
        
        queue = new CommandQueue(
            new FollowerCommand(drive, E5_TO_BM),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
