package frc.robot.autonomous.blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.autonomous.BaseAuto;
import frc.robot.autonomous.trajectory.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ScoreCommand.Height;

import static frc.robot.autonomous.AutonomousPositions.*;

import java.util.List;

public class BlueCableCoverThreePieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private Trajectory I5_TO_GP4 = TrajectoryGenerator.generateTrajectory(
        GRID_I.getPose(),
        List.of(
            INTERMEDIARY_CABLE_COVER.getTranslation()
        ), 
        GAME_PIECE_4.getPose(), 
        config.setReversed(false)
    );

    private Trajectory GP4_TO_I5 = TrajectoryGenerator.generateTrajectory(
        GAME_PIECE_4.getPose(),
        List.of(
            INTERMEDIARY_CABLE_COVER.getTranslation()
        ), 
        GRID_I.getPose(), 
        config.setReversed(true)//.setEndVelocity(1d)
    );

    private Trajectory I5_TO_GP3 = TrajectoryGenerator.generateTrajectory(
        GRID_I.getPose(),
        List.of(
            INTERMEDIARY_CABLE_COVER.getTranslation()
        ), 
        GAME_PIECE_3.getPose(), 
        config.setReversed(false)
    );

    private Trajectory GP3_TO_I5 = TrajectoryGenerator.generateTrajectory(
        GAME_PIECE_3.getPose(),
        List.of(
            INTERMEDIARY_CABLE_COVER.getTranslation()
        ), 
        GRID_G.getPose(), 
        config.setReversed(true)//.setEndVelocity(1d)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, scoreTime),
            new JointCommand(
                new FollowerCommand(drive, new SwerveTrajectory(I5_TO_GP4), Rotation2d.fromDegrees(0), "I5 -> Icc -> GP4"),
                new IntakeCommand(IntakeMode.OUT, 2d)
            ),
            new FollowerCommand(drive, new SwerveTrajectory(GP4_TO_I5), Rotation2d.fromDegrees(0), "GP4 -> Icc -> I5"),
            new ScoreCommand(Height.MID, scoreTime),
            new JointCommand(
                new FollowerCommand(drive, new SwerveTrajectory(I5_TO_GP3), Rotation2d.fromDegrees(0), "I5 -> Icc -> GP3"),
                new IntakeCommand(IntakeMode.OUT, 2d)
            ),
            new FollowerCommand(drive, new SwerveTrajectory(GP3_TO_I5), Rotation2d.fromDegrees(0), "GP3 -> Icc -> G5"),
            new ScoreCommand(Height.MID, scoreTime)
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
}