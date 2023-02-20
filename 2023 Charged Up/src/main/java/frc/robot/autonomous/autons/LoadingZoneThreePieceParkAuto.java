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
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ScoreCommand.Height;

import static frc.robot.autonomous.AutonomousPositions.*;

public class LoadingZoneThreePieceParkAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(5, 5);

    @Override
    public void initialize() {
        SwerveTrajectory A_TO_GP1 = generateTrajectoryFromPoints(
            config.setReversed(false),
            GRID_A.getPose(),
            INTERMEDIARY_LOADING_ZONE.getPose(),
            GAME_PIECE_1.getPose()
        );

        SwerveTrajectory GP1_TO_A = generateTrajectoryFromPoints(
            config.setReversed(true),
            GAME_PIECE_1.getPose(),
            INTERMEDIARY_LOADING_ZONE.getPose(),
            GRID_A.getPose()
        );

        SwerveTrajectory A_TO_GP2 = generateTrajectoryFromPoints(
            config.setReversed(false),
            GRID_A.getPose(),
            INTERMEDIARY_LOADING_ZONE.translate(0,-0.3),
            GAME_PIECE_2.getPose()
        );

        SwerveTrajectory GP2_TO_C = generateTrajectoryFromPoints(
            config.setReversed(true),
            GAME_PIECE_2.getPose(),
            INTERMEDIARY_LOADING_ZONE.translate(0,-0.3),
            COMMUNITY_LOADING_ZONE.getPose(),
            GRID_C.getPose()
        );

        SwerveTrajectory C_TO_BM = generateTrajectoryFromPoints(
            config.setReversed(false),
            GRID_C.getPose(),
            INTERMEDIARY_LOADING_ZONE.translate(-1,-0.5),
            INTERMEDIARY_LOADING_ZONE.getPose(),
            BALANCE_LOADING_ZONE.translate(1,0),
            BALANCE_MIDDLE.rotate(Rotation2d.fromDegrees(180))
        );

        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, 1.5),
            new JointCommand(
                new FollowerCommand(drive, A_TO_GP1),
                new IntakeCommand(IntakeMode.OUT, 1d)
            ),
            new FollowerCommand(drive, GP1_TO_A),
            new ScoreCommand(Height.HIGH, 0.5),
            new JointCommand(
                new FollowerCommand(drive, A_TO_GP2),
                new IntakeCommand(IntakeMode.OUT, 1d)
            ),
            new FollowerCommand(drive, GP2_TO_C),
            new ScoreCommand(Height.HIGH, 0.5),
            new FollowerCommand(drive, C_TO_BM),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
