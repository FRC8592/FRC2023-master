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

public class LoadingZoneThreePieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(5, 3);

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

        SwerveTrajectory Ilz_TO_GP2 = generateTrajectoryFromPoints(
            INTERMEDIARY_LOADING_ZONE, 
            GAME_PIECE_2, 
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(5)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        SwerveTrajectory GP2_TO_Ilz = generateTrajectoryFromPoints(
            GAME_PIECE_2, 
            INTERMEDIARY_LOADING_ZONE, 
            Rotation2d.fromDegrees(0), 
            config
                .setStartVelocity(0)
                .setEndVelocity(5)
                .setKinematics(drive.getKinematics())
                .setReversed(true)
        );

        // SwerveTrajectory A5_TO_C5 = generateTrajectoryFromPoints(
        //     GRID_A, 
        //     GRID_C,
        //     Rotation2d.fromDegrees(0),
        //     Rotation2d.fromDegrees(-90),
        //     Rotation2d.fromDegrees(-90),
        //     config
        //         .setStartVelocity(0)
        //         .setEndVelocity(0)
        //         .setKinematics(drive.getKinematics())
        //         .setReversed(true)
        // );

        SwerveTrajectory Ilz_TO_C5 = generateTrajectoryFromPoints(
            config
                .setStartVelocity(5)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(true),
            INTERMEDIARY_LOADING_ZONE,
            COMMUNITY_LOADING_ZONE,
            GRID_C
        );

        queue = new CommandQueue(
            new ScoreCommand(intake), // Score preload
            new FollowerCommand(drive, A5_TO_Ilz), // Go and grab second game piece
            new JointCommand(
                new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake)
            ),
            new FollowerCommand(drive, GP1_TO_Ilz), // Drive back to grid space A
                new FollowerCommand(drive, Ilz_TO_A5),
            // new ScoreCommand(Height.MID), // Score mid
            new FollowerCommand(drive, A5_TO_Ilz), // Go and grab third game piece
                new JointCommand(
                    new FollowerCommand(drive, Ilz_TO_GP2),
                    new IntakeCommand(intake)
                ),
            new FollowerCommand(drive, GP2_TO_Ilz), // Drive back to grid space C
                new FollowerCommand(drive, Ilz_TO_C5.addRotation(Rotation2d.fromDegrees(180))),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}