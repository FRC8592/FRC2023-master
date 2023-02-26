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
        SwerveTrajectory TEST_1 = generateTrajectoryFromPoints(
            config.setStartVelocity(0.0).setEndVelocity(0.0),//.setReversed(true),
            GRID_I.getPose(),
            GRID_I.translate(1, 0)
        );

        SwerveTrajectory TEST_2 = generateTrajectoryFromPoints(
            config.setStartVelocity(0.0).setEndVelocity(0.0),
            GRID_I.translate(1, 0),
            GRID_I.translate(-1, 0)
        );

        queue = new CommandQueue(
            new DelayCommand(0.5),
            new FollowerCommand(drive, TEST_1)
            // new FollowerCommand(drive, TEST_1.addRotation(Rotation2d.fromDegrees(-90)))
            // new FollowerCommand(drive, TEST_2)
            // new FollowerCommand(drive, TEST_2.addRotation(Rotation2d.fromDegrees(90)))
            // new FollowerCommand(drive, I5_TO_Icc.addRotation(Rotation2d.fromDegrees(-90)))
                // new FollowerCommand(drive, Icc_TO_GP3)
            // new FollowerCommand(drive, I5_TO_Icc.addRotation(Rotation2d.fromDegrees(90)))
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
