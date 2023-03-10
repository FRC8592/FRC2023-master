package frc.robot.autonomous.autons.other;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class MoveOutCommunityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory MOVE_OUT = generateTrajectoryFromPoints(
        config,
        GRID_A.getPose(),
        GRID_A.translate(4, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, MOVE_OUT.addRotation(Rotation2d.fromDegrees(180), Math.PI, 1.0))
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}