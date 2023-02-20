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

/**
 * Make sure to add grabbing the third piece before climbing
 */
public class CableCoverTwoPieceParkAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(5, 3);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);

    @Override
    public void initialize() {
        SwerveTrajectory I5_TO_Icc = generateTrajectoryFromPoints(
            GRID_I, 
            INTERMEDIARY_CABLE_COVER,
            Rotation2d.fromDegrees(0), 
            slowConfig
                .setStartVelocity(0)
                .setEndVelocity(2)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        SwerveTrajectory Icc_TO_GP4 = generateTrajectoryFromPoints(
            INTERMEDIARY_CABLE_COVER,
            GAME_PIECE_4,
            Rotation2d.fromDegrees(180), 
            fastConfig
                .setStartVelocity(2)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        SwerveTrajectory GP4_TO_Icc = generateTrajectoryFromPoints(
            GAME_PIECE_4,
            INTERMEDIARY_CABLE_COVER,
            Rotation2d.fromDegrees(0), 
            fastConfig
                .setStartVelocity(0)
                .setEndVelocity(3)
                .setKinematics(drive.getKinematics())
                .setReversed(true)
        );

        SwerveTrajectory Icc_TO_I5 = generateTrajectoryFromPoints(
            INTERMEDIARY_CABLE_COVER,
            GRID_I,
            Rotation2d.fromDegrees(0), 
            slowConfig
                .setStartVelocity(3)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(true)
        );
        
        SwerveTrajectory Icc_TO_BM = generateTrajectoryFromPoints(
            INTERMEDIARY_CABLE_COVER,
            BALANCE_MIDDLE,
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(180), 
            slowConfig
                .setStartVelocity(2)
                .setEndVelocity(0)
                .setKinematics(drive.getKinematics())
                .setReversed(false)
        );

        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, 1.5),
            new FollowerCommand(drive, I5_TO_Icc),
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_GP4),
                new IntakeCommand(IntakeMode.OUT)
            ),
            new FollowerCommand(drive, GP4_TO_Icc),
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_I5),
                new ScoreCommand(Height.MID)
            ),
            new FollowerCommand(drive, I5_TO_Icc),
                new FollowerCommand(drive, Icc_TO_BM),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}