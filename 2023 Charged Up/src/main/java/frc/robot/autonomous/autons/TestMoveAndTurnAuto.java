package frc.robot.autonomous.autons;

import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class TestMoveAndTurnAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    @Override
    public void initialize() {
        SwerveTrajectory I5_TO_Icc = generateTrajectoryFromPoints(
            // config.setStartVelocity(0.0).setEndVelocity(1.0),
            config.setStartVelocity(0.0).setEndVelocity(0.0),
            GRID_I.getPose(),
            INTERMEDIARY_CABLE_COVER.getPose()
        );

        SwerveTrajectory Icc_TO_GP3 = generateTrajectoryFromPoints(
            config.setStartVelocity(0.0).setEndVelocity(0.0),
            INTERMEDIARY_CABLE_COVER.getPose(),
            GAME_PIECE_3.getPose()
        );

        queue = new CommandQueue(
            new DelayCommand(0.5),
            new FollowerCommand(drive, I5_TO_Icc.addRotation(Rotation2d.fromDegrees(180)))
                // new FollowerCommand(drive, Icc_TO_GP3)
            // new FollowerCommand(drive, I5_TO_Icc.addRotation(Rotation2d.fromDegrees(90)))
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
