package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

/**
 * Make sure to add grabbing the third piece before climbing
 */
public class LoadingZoneTwoPieceParkAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(5, 5);

    @Override
    public void initialize() {
        SwerveTrajectory A5_TO_Ilz = generateTrajectoryFromPoints(
            GRID_A, 
            INTERMEDIARY_LOADING_ZONE,
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(0)
                .setEndVelocity(5)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
            INTERMEDIARY_LOADING_ZONE,
            GAME_PIECE_1,
            Rotation2d.fromDegrees(180), 
            config
                .setStartVelocity(5)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        SwerveTrajectory GP1_TO_Ilz = generateTrajectoryFromPoints(
            GAME_PIECE_1,
            INTERMEDIARY_LOADING_ZONE,
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(0)
                .setEndVelocity(5)
                .setKinematics(drive.getKinematics())
                .setReversed(true)
        );

        SwerveTrajectory Ilz_TO_A5 = generateTrajectoryFromPoints(
            INTERMEDIARY_LOADING_ZONE,
            GRID_A,
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(5)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(true)
        );
        
        SwerveTrajectory Ilz_TO_BM = generateTrajectoryFromPoints(
            INTERMEDIARY_LOADING_ZONE,
            BALANCE_MIDDLE,
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(5)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        queue = new CommandQueue(
            new FollowerCommand(drive, A5_TO_Ilz),
            new JointCommand(
                new FollowerCommand(drive, Ilz_TO_GP1),
                new IntakeCommand(intake)
            ),
            new FollowerCommand(drive, GP1_TO_Ilz),
            new JointCommand(
                new FollowerCommand(drive, Ilz_TO_A5)
            ),
            new FollowerCommand(drive, A5_TO_Ilz),
                new FollowerCommand(drive, Ilz_TO_BM),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}