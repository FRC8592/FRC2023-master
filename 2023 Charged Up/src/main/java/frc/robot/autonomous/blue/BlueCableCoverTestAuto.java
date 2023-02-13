package frc.robot.autonomous.blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.BaseAuto;
import frc.robot.autonomous.trajectory.SwerveTrajectory;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ScoreCommand.Height;

import static frc.robot.autonomous.AutonomousPositions.*;


public class BlueCableCoverTestAuto extends BaseAuto {
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 1);
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3, 1);

    @Override
    public void initialize() {
        SwerveTrajectory I5_TO_Icc = generateTrajectoryFromPoints(
            GRID_I, 
            INTERMEDIARY_CABLE_COVER,
            Rotation2d.fromDegrees(0), 
            slowConfig
            .setStartVelocity(0)
            .setEndVelocity(1)
            .setReversed(false)
            .setKinematics(drive.getKinematics())
        );

        SwerveTrajectory Icc_TO_GP4 = generateTrajectoryFromPoints(
            INTERMEDIARY_CABLE_COVER,
            GAME_PIECE_4,
            Rotation2d.fromDegrees(0), 
            fastConfig
            .setStartVelocity(1)
            .setEndVelocity(0)
            .setReversed(false)
            .setKinematics(drive.getKinematics())
        );

        SwerveTrajectory GP4_TO_BM = generateTrajectoryFromPoints(
            GAME_PIECE_4, 
            B_MIDDLE, 
            Rotation2d.fromDegrees(0), 
            fastConfig
            .setStartVelocity(0)
            .setEndVelocity(0)
            .setReversed(true)
            .setKinematics(drive.getKinematics())
        );

        SwerveTrajectory TEST_2_METERS = generateTrajectoryFromPoints(
            TEST_1, 
            TEST_2, 
            Rotation2d.fromDegrees(0), 
            fastConfig
                .setStartVelocity(0)
                .setEndVelocity(0)
                .setReversed(false)
                .setKinematics(drive.getKinematics())
        );

        queue = new CommandQueue(
            // new ScoreCommand(Height.HIGH),
            // new FollowerCommand(drive, I5_TO_Icc),
            // new JointCommand(
            //     new FollowerCommand(drive, Icc_TO_GP4),
            //     new IntakeCommand(IntakeMode.OUT)
            // ),
            // new FollowerCommand(drive, GP4_TO_BM),
            // new AutobalanceCommand(drive)
            new FollowerCommand(drive, TEST_2_METERS)
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
